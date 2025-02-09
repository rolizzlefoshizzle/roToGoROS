# license removed for brevity
import rospy
from std_msgs.msg import String, Bool, Float64MultiArray
from matplotlib import pyplot as plt
from matplotlib import patches as patches
import numpy as np


class plotter:
    """This class plots the execution and planning of the system
    - It subscribes to
        - global plan
        - global plan satisfaction boolean
        - local plan
        - truther
        - commanded acceleration"""

    def __init__(self):
        #########################
        # initialize parameters #
        #########################
        self.programRunning = True
        # robot position
        self.roboPos = [rospy.get_param("/x0"), rospy.get_param("/y0")]

        # empty robot trajectory
        self.roboTrajectory = np.array([]).reshape(0, 2)

        # predicate positions
        self.pred = [rospy.get_param("/xe0"), rospy.get_param("/ye0")]

        # empty global plan
        self.longPlan = np.array([]).reshape(0, 2)

        # empty local plan
        self.shortPlan = np.array([]).reshape(0, 2)

        # global plan style
        self.longLinestyle = '-'

        # execution time
        self.time = 0

        # initial input
        self.input = np.array([[0, 0]]).T

        # nominal input
        self.nominalInput = np.array([[0, 0]]).T

        # boolean to plot inputs
        self.plotAccels = rospy.get_param("/plotAccel")

        # initialize node
        rospy.init_node('plotter', anonymous=True)

        ###############################
        # initialize node information #
        ###############################
        # define subscriber callbacks
        rospy.Subscriber('goals', Float64MultiArray, self.get_long_plan)
        rospy.Subscriber('traj', Float64MultiArray, self.get_short_plan)
        rospy.Subscriber('observations', Float64MultiArray,
                         self.observe)
        rospy.Subscriber('highPlanGood', Bool, self.high_plan_boolean)
        rospy.Subscriber('cmdAcc', Float64MultiArray, self.received_input)
        rospy.Subscriber('planning', Bool, self.check_planning)

    def check_planning(self, data):
        if (not data.data):
            self.programRunning = False

    def received_input(self, data):
        """receive and save commanded input"""
        self.input = np.array(data.data[0:2]).reshape(2, 1)
        self.input = (0.5/np.linalg.norm(self.input))*self.input
        self.nominalInput = np.array(data.data[2:]).reshape(2, 1)
        self.nominalInput = (
            0.5/np.linalg.norm(self.nominalInput))*self.nominalInput

    def observe(self, data):
        """receive and save true robot state, predicate state, and time"""
        self.pred = data.data[5:]
        self.roboPos = data.data[1:3]
        self.roboTrajectory = np.vstack((
            self.roboTrajectory, np.array(self.roboPos).reshape(1, 2)))
        self.time = data.data[0]

    def get_long_plan(self, data):
        """receive and save global plan"""
        self.longPlan = np.reshape(
            data.data, [round(len(data.data)/7), 7])
        self.longPlan = self.longPlan[:, 1:]

    def get_short_plan(self, data):
        """receive and save local plan"""
        self.shortPlan = np.reshape(
            data.data, [round(len(data.data)/7), 7])
        self.shortPlan = self.shortPlan[:, 1:3]

    def high_plan_boolean(self, data):
        """receive and save global plan validity boolean"""
        if data.data:
            self.longLinestyle = '-'
        else:
            self.longLinestyle = '--'

    def run(self):
        """plot the execution live"""
        plt.ion()
        plt.show()
        axis = plt.axes(xlim=(0, 5), ylim=(0, 5))
        point, = axis.plot([], [], 'bo')
        longLine, = axis.plot([], [], 'c-')
        shortLine, = axis.plot([], [], 'g-')
        trajectory, = axis.plot([], [], 'b-')
        obs = patches.Circle((0, 0), radius=0.5, facecolor="black")
        goal = patches.Rectangle((4, 4), 1, 1, facecolor="green")
        other = patches.Rectangle((4, 0), 1, 1, facecolor="gray")
        if self.plotAccels:
            a = axis.arrow(self.roboPos[0], self.roboPos[1],
                           self.input[0][0], self.input[1][0], width=0.05)
            b = axis.arrow(self.roboPos[0], self.roboPos[1],
                           self.nominalInput[0][0], self.nominalInput[1][0], ls='--', width=0.05)
        axis.add_patch(goal)
        axis.add_patch(obs)
        axis.add_patch(other)
        # define rate
        rate = rospy.Rate(50)
        # main loop
        while not rospy.is_shutdown() and self.programRunning:
            trajectory.set_data(self.roboTrajectory[:, 0],
                                self.roboTrajectory[:, 1])
            point.set_data([self.roboPos[0]], [self.roboPos[1]])
            longLine.set_data(self.longPlan[:, 0], self.longPlan[:, 1])
            longLine.set_linestyle(self.longLinestyle)
            shortLine.set_data(self.shortPlan[:, 0], self.shortPlan[:, 1])
            if self.plotAccels:
                a.remove()
                b.remove()
                a = axis.arrow(self.roboPos[0], self.roboPos[1],
                               self.input[0][0], self.input[1][0], width=0.05)
                b = axis.arrow(self.roboPos[0], self.roboPos[1],
                               self.nominalInput[0][0], self.nominalInput[1][0], ls='--', width=0.05)
            obs.set(center=(self.pred[0], self.pred[1]))
            axis.set_title(round(self.time, 1))

            plt.draw()
            plt.pause(0.00000000001)

            rate.sleep()


if __name__ == '__main__':
    try:
        myPlotter = plotter()
        myPlotter.run()
    except rospy.ROSInterruptException:
        pass
