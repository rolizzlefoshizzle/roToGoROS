import rospy
import numpy as np
from std_msgs.msg import String, Float64MultiArray, Bool


class truther:
    """This class simulates the true state of the system
    - It subscribes to the commanded input and the planning boolean
    - It uses the commanded input to simulate the robot and predicate dynamics
    - It publishes to the observation topic /"truther" """

    def __init__(self):
        #########################
        # initialize parameters #
        #########################
        # predicate position
        self.programRunning = True

        self.predicatePosition = np.array(
            [rospy.get_param("/xe0"), rospy.get_param("/ye0")])

        # define workspace bounds
        self.bounds = np.array([rospy.get_param("/xBoundLow"), rospy.get_param("/xBoundHigh"),
                                rospy.get_param("/yBoundLow"), rospy.get_param("/yBoundHigh")])

        # robot position and velocity
        self.roboState = np.array([[rospy.get_param(
            "/x0"), rospy.get_param("/y0"), rospy.get_param("/vx0"), rospy.get_param("/vy0")]]).T

        # variance of gaussian disturbance acting on predicates
        self.sigma = rospy.get_param("/predicateSigma")  # get from a rosparam

        # data to publish
        self.dataToPublish = Float64MultiArray()

        # disable until planning is enabled
        self.enabled = 0

        # the resolution of the time discretization of the sytem
        self.dt = rospy.get_param("/truthDt")

        # input
        self.input = np.array([[0], [1]])

        # system dynamics' matrices
        self.A = np.array(
            [[1, 0, self.dt, 0], [0, 1, 0, self.dt], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.B = np.array(
            [[0.5*self.dt*self.dt, 0], [0, 0.5*self.dt*self.dt], [self.dt, 0], [0, self.dt]])

        # initial time
        self.startTime = 0

        ###############################
        # initialize node information #
        ###############################
        # define topics to publish to
        self.observationTopic = rospy.Publisher(
            'observations', Float64MultiArray, queue_size=10)

        # initialize node
        rospy.init_node('predicateManager', anonymous=True)

        # define subscriber callbacks
        rospy.Subscriber('cmdAcc', Float64MultiArray, self.received_input)
        rospy.Subscriber('planning', Bool, self.check_planning)

    def received_input(self, data):
        """receive and save control input"""
        if self.startTime == 0:  # todo: move this to robot callback
            self.enabled = 1
            self.startTime = rospy.get_time()
        self.input = np.array(data.data[0:2]).reshape(2, 1)

    def check_planning(self, data):
        """disable truther when planning is over"""
        if self.enabled and ~data.data:
            self.enabled = 0
            self.programRunning = False

    def run(self):
        """simulate the dynamics, disturb predicates"""
        # define rate
        rate = rospy.Rate(round(1/rospy.get_param("/truthDt")))

        # main loop
        while not rospy.is_shutdown() and self.programRunning:
            if self.enabled:
                # publish my "truth" - time, roboPos, predicatePos
                self.dataToPublish.data = np.hstack((rospy.get_time(
                )-self.startTime, self.roboState.T[0], self.predicatePosition)).tolist()

                # publish predicate position
                self.observationTopic.publish(self.dataToPublish)

                # calculate roboPos using input
                self.roboState = np.matmul(
                    self.A, self.roboState) + np.matmul(self.B, self.input)

                # calculate predicatePos using sigma
                velocities = np.random.normal(0, self.sigma, size=(1, 2))
                self.predicatePosition = self.predicatePosition + \
                    np.array(
                        [self.dt*velocities[0, 0], self.dt*velocities[0, 1]])

                # correct the predicates to enforce bounds
                if self.predicatePosition[0]-1 < self.bounds[0]:
                    self.predicatePosition[0] = self.bounds[0] + 1
                elif self.predicatePosition[0] + 1 > self.bounds[1]:
                    self.predicatePosition[0] = self.bounds[1] - 1
                if self.predicatePosition[1]-1 < self.bounds[2]:
                    self.predicatePosition[1] = self.bounds[2] + 1
                elif self.predicatePosition[1]+1 > self.bounds[3]:
                    self.predicatePosition[1] = self.bounds[3] - 1

            rate.sleep()


if __name__ == '__main__':
    try:
        myTruther = truther()
        myTruther.run()
    except rospy.ROSInterruptException:
        pass
