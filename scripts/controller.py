#!/usr/bin/env python
import rospy
import numpy as np
from control import dlqr
from std_msgs.msg import Float64MultiArray, Bool


class executor:

    """This class represents a node in charge of publishing control inputs for
    the hardware to receive
    - It subscribes to the local planner's trajectory and the true state of the
      system
    - It uses an LQR tracking controller to follow the local planner's
      trajectory
    - It publishes to the commanded acceleration topic"""

    def __init__(self):
        #########################
        # initialize parameters #
        #########################
        self.programRunning = True

        # data to publish
        self.dataToPublish = Float64MultiArray()

        # disable until receiving trajectory
        self.enabled = 0

        # actual time, used to find relevant point on local planner trajectory
        self.trueTime = 0

        # initial robot state
        self.roboState = np.array([rospy.get_param("/x0"), rospy.get_param("/y0"),
                                   rospy.get_param("/vx0"), rospy.get_param("/vy0")])

        # time resolution with which to discretize the system
        self.dt = rospy.get_param("/executionDt")

        # time resolution with which to discretize the system
        self.track = rospy.get_param("/trackingController")

        # define the A and B matrices of the DT linear system
        self.A = np.array(
            [[1, 0, self.dt, 0], [0, 1, 0, self.dt], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.B = np.array(
            [[0.5*self.dt*self.dt, 0], [0, 0.5*self.dt*self.dt], [self.dt, 0], [0, self.dt]])
        # The state error and control input penalties for the LQR controller
        R = rospy.get_param("/controllerR")*np.eye(2)
        Q = rospy.get_param("/controllerQ")*np.eye(4)

        # the gain matrix from LQR control
        self.K, _, _ = dlqr(self.A, self.B, Q, R)

        ###############################
        # initialize node information #
        ###############################
        # define topics to publish to
        self.inputTopic = rospy.Publisher(
            'cmdAcc', Float64MultiArray, queue_size=10)

        # initialize node
        rospy.init_node('executor', anonymous=True)

        # define subscriber callbacks
        rospy.Subscriber('traj', Float64MultiArray,
                         self.process_traj)
        rospy.Subscriber('observations', Float64MultiArray,
                         self.measure_truth)
        rospy.Subscriber('planning', Bool, self.check_planning)

    def check_planning(self, data):
        """receive and store planning boolean"""
        self.planning = data.data
        if (not data.data):
            self.programRunning = False
        # todo: updates class's attributes representing predicates

    def process_traj(self, data):
        """receive and save trajectory"""
        self.enabled = 1
        self.trajectory = np.reshape(
            data.data, [round(len(data.data)/7), 7])

    def measure_truth(self, data):
        """receive and save true state"""
        self.trueTime = data.data[0]
        self.roboState = np.array(data.data[1:5])

    def run(self):
        """implement LQR tracking controller"""
        # define rate
        rate = rospy.Rate(round(1/rospy.get_param("/executionDt")))

        # main loop
        while not rospy.is_shutdown() and self.programRunning:
            if self.enabled and self.planning:
                # find index in local plan corresponding to current time
                index = np.searchsorted(
                    self.trajectory[:, 0], self.trueTime)-1
                if index == (np.shape(self.trajectory)[0] - 1):
                    rospy.loginfo("ran out of trajectory to track!")

                # use lqr to find additional control input
                error = self.trajectory[index, 1:5] - self.roboState
                correction = np.matmul(self.K, error)

                # publish the final and nominal control input
                nominal = self.trajectory[index, 5:]

                if self.track:
                    self.dataToPublish.data = (np.hstack((
                        correction + nominal, nominal))).tolist()
                else:
                    self.dataToPublish.data = (np.hstack((
                        nominal, nominal))).tolist()

                self.inputTopic.publish(self.dataToPublish)

            rate.sleep()


if __name__ == '__main__':
    try:
        myExecutor = executor()
        myExecutor.run()
    except rospy.ROSInterruptException:
        pass
