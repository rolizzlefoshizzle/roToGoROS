# license removed for brevity
from vpsto.vpsto import VPSTO, VPSTOOptions
import rospy
import numpy as np
from std_msgs.msg import String, Float64MultiArray, Bool, Float64
import stlrom
import time

# todo: add useful info for debugging


class Loss():
    """This class represents the loss function for the VP-Sto Planner"""

    def __init__(self, vpsto, bounds, dt, T_max, history, predSigs, startTime, boundLossWeight, cppCost, shortHorz, pos_key='pos', vel_key='vel'):
        self.bounds = bounds
        self.pos_key = pos_key
        self.vel_key = vel_key
        self.vpsto = vpsto
        self.history = history
        self.predSigs = predSigs
        self.startTime = startTime
        self.boundLossWeight = boundLossWeight
        self.driver = stlrom.STLDriver()
        self.pop_size = self.vpsto.opt.pop_size
        self.cppCost = cppCost
        if shortHorz:
            self._N_eval_max = round(T_max / dt) + 1
            self.t = np.linspace(0, T_max, self._N_eval_max)
            self.robsInd = 2
        else:
            self._N_eval_max = round((T_max-self.startTime) / dt) + 1
            self.t = np.linspace(
                0, T_max-self.startTime, self._N_eval_max)
            self.robsInd = 0

    def equalize_pop_time_resolution(self, pop):
        via_points = pop['p_via']
        x0 = pop[self.pos_key][:, 0]
        dx0 = pop[self.vel_key][:, 0]
        dxT = np.zeros_like(dx0)
        T = pop['T']
        pop_size = x0.shape[0]
        t_batch = np.tile(self.t, (pop_size, 1))
        s_batch = t_batch / T[:, None]
        s_batch_vec = s_batch.flatten()

        Phi_batch_vec_jax, _, _ = self.vpsto.vptraj.obf.get_basis_jax(
            s_batch_vec)
        Phi_batch_vec = np.array(Phi_batch_vec_jax)

        Phi_batch = Phi_batch_vec.reshape(
            (pop_size, -1, Phi_batch_vec.shape[-1]))
        w = np.concatenate((x0, via_points, dx0, dxT), axis=-1)
        w[:, -4:] *= T[:, None]
        q_batch = np.sum(Phi_batch * w[:, None, :], axis=-1)
        new_pop = q_batch.reshape((pop_size, self._N_eval_max, -1))
        return new_pop

    def loss_stl(self, pop):
        # find the loss from violating the STL formula
        pop_ = self.equalize_pop_time_resolution(pop)
        costs = []
        t = self.t + self.startTime
        self.t_eval = np.reshape(t, [t.size, 1])
        for i in range(pop_.shape[0]):
            costs.append(self.candidate_stl_loss(pop_[i]))
        costs = np.array(costs)
        return np.array(costs)

    def loss_stl_cpp(self, pop):
        # find the loss from violating the STL formula
        pop_ = self.equalize_pop_time_resolution(pop)
        costs = []
        t = self.t + self.startTime
        self.t_eval = np.reshape(t, [t.size, 1])
        tiledPreds2d = np.tile(self.predSigs, (t.shape[0], 1))
        tiledPreds3d = np.tile(tiledPreds2d, (self.pop_size, 1, 1))
        t_tiled = np.tile(t.reshape(t.shape[0], 1), (self.pop_size, 1, 1))
        futureSignalTiled = np.dstack((t_tiled, pop_, tiledPreds3d))
        historyTiled = np.tile(self.history, (self.pop_size, 1, 1))
        totalSignalTiled = np.hstack((historyTiled, futureSignalTiled))
        costs = stlrom.batch_find_robustness(
            self.driver, totalSignalTiled, "phi", self.robsInd)
        costs = np.array(costs)
        return np.array(costs)

    def candidate_stl_loss(self, candidate):
        # just the future states
        stateSignal = np.hstack((self.t_eval, candidate))
        # future states and predicates (assumed constant)
        futureSignal = np.hstack((stateSignal, np.tile(
            self.predSigs, (stateSignal.shape[0], 1))))
        # past and present, states and predicates
        signal = np.vstack((self.history, futureSignal))
        self.driver.data = signal
        robs = self.driver.get_online_rob("phi")
        # maximize upper bound for short horizon mpc
        return -robs[self.robsInd]

    def loss_limits(self, pop):
        # find the loss from violating the bounds of the workspace
        q = pop[self.pos_key]
        d_min = np.maximum(np.zeros_like(q), self.bounds[:, 0] - q)
        d_max = np.maximum(np.zeros_like(q), q - self.bounds[:, 1])
        return np.sum(d_min > 0.0, axis=(1, 2)) + np.sum(d_max > 0.0, axis=(1, 2))

    def loss_full_horizon(self, pop):
        # find the total loss
        cost_limits = self.loss_limits(pop)
        # cost_stl = self.loss_stl_particles(pop)
        if self.cppCost:
            cost_stl = self.loss_stl_cpp(pop)
        else:
            cost_stl = self.loss_stl(pop)
        cost = self.boundLossWeight * cost_limits + cost_stl
        return cost


class planner:
    """This class represents a node in charge of finding the global plan
    - It subscribes to
        - the user command for the formula
        - the true state for keeping track of signal history
        - the planning command to start and stop planning
    - It finds trajectories for the full horizon using VP-Sto
    - It publishes the global plan trajectory and a boolean specifying whether
      the global plan satisfies the STL formula"""

    def __init__(self):
        #########################
        # initialize parameters #
        #########################

        # disable planning until told otherwise
        self.planning = 0

        self.programRunning = True

        # initialize empty true history
        self.trueHistory = np.array([]).reshape(0, 7)

        # initial robot position
        self.startPos = np.array(
            [rospy.get_param("/x0"), rospy.get_param("/y0")])

        # initial robot velocity
        self.startVel = np.array(
            [rospy.get_param("/vx0"), rospy.get_param("/vy0")])

        # get short Horz boolean
        self.shortHorz = rospy.get_param("/useFiniteHorizon")

        # initial predicate state
        pred0 = np.array(
            [rospy.get_param("/xe0"), rospy.get_param("/ye0")])

        # define vp-sto options
        opt = VPSTOOptions(ndof=2)
        opt.vel_lim = np.array(
            [rospy.get_param("/xVelLim"), rospy.get_param("/yVelLim")])
        opt.acc_lim = np.array(
            [rospy.get_param("/xAccLim"), rospy.get_param("/yAccLim")])
        if self.shortHorz:
            opt.N_via = rospy.get_param("/localViaPoints")
            opt.pop_size = rospy.get_param("/localPopSize")
            opt.sigma_init = rospy.get_param("/localSigmaInit")
            opt.max_iter = rospy.get_param("/localMaxIter")
            opt.verbose = rospy.get_param("/localVerbose")
        else:
            opt.N_via = rospy.get_param("/globalViaPoints")
            opt.pop_size = rospy.get_param("/globalPopSize")
            opt.sigma_init = rospy.get_param("/globalSigmaInit")
            opt.max_iter = rospy.get_param("/globalMaxIter")
            opt.verbose = rospy.get_param("/globalVerbose")

        # create vp-sto class
        self.vpsto = VPSTO(opt)

        # define workspace bounds
        bounds = np.array([[rospy.get_param("/xBoundLow"), rospy.get_param("/xBoundHigh")],
                          [rospy.get_param("/yBoundLow"), rospy.get_param("/yBoundHigh")]])
        # dt for evaluating cost
        self.dt = rospy.get_param("/stlEvaluationDt")
        # dt for replanning
        if self.shortHorz:
            self.mpcDt = rospy.get_param("/shortHorzDt")
        else:
            self.mpcDt = rospy.get_param("/longHorzDt")
        # dt for output resolution
        self.executionDt = rospy.get_param(
            "/executionDt")

        # define T max of planner (time horizon of STL formulae)
        if self.shortHorz:
            self.Tmax = rospy.get_param("/localTmax")
        else:
            self.Tmax = rospy.get_param("/globalTmax")

        # get boundary violation loss weight
        boundLossWeight = rospy.get_param("/boundLossWeight")

        # decide whether to use cpp to find cost or not
        useCppCost = rospy.get_param("/cppFindCost")

        # define empty initial history for loss class
        history = np.array([]).reshape(0, 5)

        # create loss class
        self.loss = Loss(self.vpsto, bounds, self.dt, self.Tmax, history,
                         pred0, 0, boundLossWeight, useCppCost, self.shortHorz)

        # initialize "current time"
        self.realTime = 0

        # data to publish
        self.arrayDataToPublish = Float64MultiArray()
        self.floatDataToPublish = Float64()

        ###############################
        # initialize node information #
        ###############################
        # define topics to publish to
        self.goalTopic = rospy.Publisher(
            'traj', Float64MultiArray, queue_size=10)
        self.logExecTime = rospy.Publisher(
            'logExecTime', Float64, queue_size=10)
        self.logPlanTime = rospy.Publisher(
            'logPlanTime', Float64, queue_size=10)
        self.logCost = rospy.Publisher(
            'logCost', Float64, queue_size=10)

        # initialize node
        rospy.init_node('planner', anonymous=True)

        # define subscriber callbacks
        rospy.Subscriber('remainingFormula', String,
                         self.process_remaining_formula)
        rospy.Subscriber('stateObservations', Float64MultiArray,
                         self.observe)
        rospy.Subscriber('planning', Bool, self.check_planning)

    def process_remaining_formula(self, data):
        """save the most recent STL formula-to-go"""
        success = self.loss.driver.parse_string(data.data)
        while (not success):
            success = self.loss.driver.parse_string(data.data)

    def observe(self, data):
        """receive and store the true state of the system"""
        self.trueHistory = np.vstack(
            (self.trueHistory, np.reshape(np.array(data.data), [1, 7])))

    def check_planning(self, data):
        """receive and store planning boolean"""
        self.planning = data.data
        if (not data.data):
            self.programRunning = False

    def run(self):
        """vp-sto for global plan"""
        # define rate
        rate = rospy.Rate(1/self.mpcDt)

        while not rospy.is_shutdown() and self.programRunning:
            if self.planning:
                ########################
                # Update planning info #
                ########################
                # take a snapshot of the current real history
                snapshot = np.copy(self.trueHistory)

                # # if not first plan, update the initial info using the true history
                size = np.shape(snapshot)[0]
                if size > 0:
                    self.loss.predSigs = snapshot[-1, 5:]
                    self.loss.history = np.hstack(
                        (snapshot[:-1, 0:3], snapshot[:-1, 5:]))
                    # update everything for next loop
                    self.startPos = snapshot[-1, 1:3]
                    self.startVel = snapshot[-1, 3:5]
                    self.realTime = snapshot[-1, 0]

                    # update loss class time and current predicates
                    self.loss.startTime = self.realTime

                ################################
                # Find Trajectory using VP-Sto #
                ################################
                startTime = time.time()
                sol = self.vpsto.minimize(
                    self.loss.loss_full_horizon, self.startPos,
                    dq0=self.startVel,
                    dqT=np.zeros_like(self.startPos)
                )
                planTime = time.time() - startTime
                ##################
                # Process Output #
                ##################
                # get Tmax
                T = self.Tmax
                # retrieve solution trajectory
                if self.shortHorz:
                    _N_eval_max = round(T/self.executionDt) + 1
                    # t vector for extracting trajectory
                    t = np.linspace(0, T, _N_eval_max)
                else:
                    _N_eval_max = round(
                        (T-self.realTime) / self.executionDt) + 1
                    # t vector for extracting trajectory
                    t = np.linspace(0, T-self.realTime, _N_eval_max)
                X, V, A = sol.get_posvelacc(t)
                # t vector for monitoring
                if self.shortHorz:
                    t = np.linspace(self.realTime, self.realTime+T,
                                    _N_eval_max)
                else:
                    t = np.linspace(self.realTime, T,
                                    _N_eval_max)
                t = np.reshape(t, [t.size, 1])

                # publish trajectory
                trajectory = np.hstack((t, X, V, A))
                self.arrayDataToPublish.data = np.reshape(
                    trajectory, [np.shape(trajectory)[0]*np.shape(trajectory)[1]]).tolist()
                self.goalTopic.publish(self.arrayDataToPublish)

                # use cost to publish "good plan boolean" and warm start next
                # plan
                # rospy.loginfo(sol.c_best)
                if (sol.c_best < 0) & (rospy.get_param("/useWarmStart")):
                    via_indices = np.linspace(
                        0, len(X)-1, self.vpsto.opt.N_via+1, dtype=int)[1:]
                    mu_ref = X[via_indices].flatten()
                    self.vpsto.set_initial_guess(mu_ref)
                    self.vpsto.opt.sigma_init = rospy.get_param(
                        "/globalSigmaWarmStart")
                else:
                    self.vpsto.opt.sigma_init = rospy.get_param(
                        "/globalSigmaInit")
                self.floatDataToPublish.data = self.realTime
                self.logExecTime.publish(self.floatDataToPublish)
                self.floatDataToPublish.data = planTime
                self.logPlanTime.publish(self.floatDataToPublish)
                self.floatDataToPublish.data = sol.c_best
                self.logCost.publish(self.floatDataToPublish)

            else:
                rate.sleep()


if __name__ == '__main__':
    try:
        myPlanner = planner()
        myPlanner.run()
    except rospy.ROSInterruptException:
        pass
