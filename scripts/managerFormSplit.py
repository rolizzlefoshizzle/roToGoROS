#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from std_msgs.msg import String, Bool, Float64MultiArray, Float64
import stlrom
import rcpe
import math

# todo: add cleanup process to allow multiple motion planning runs


def initialize_driver(formulaStr, circRadius):
    testMonitor = rcpe.rcpeManager(
        "signal x, y, xe, ye")
    testMonitor.add_predicate("x[t]>0")
    testMonitor.add_predicate("x[t]>0.5")
    testMonitor.add_predicate("x[t]>4")
    testMonitor.add_predicate("(2*x[t])>8")
    testMonitor.add_predicate("x[t]<5")
    testMonitor.add_predicate("x[t]<1")
    testMonitor.add_predicate("(2*x[t])<10")
    testMonitor.add_predicate("y[t]>0")
    testMonitor.add_predicate("y[t]>2")
    testMonitor.add_predicate("y[t]>4")
    testMonitor.add_predicate("(2*y[t])>4")
    testMonitor.add_predicate("y[t]<5")
    testMonitor.add_predicate("y[t]<2.4")
    testMonitor.add_predicate("y[t]>2.6")
    testMonitor.add_predicate("y[t]<5")
    testMonitor.add_predicate("y[t]<3")
    testMonitor.add_predicate("(2*y[t])<6")
    testMonitor.add_predicate("y[t]<1")
    testMonitor.addSubform(
        "((x[t]>0.5)&(y[t]>0))&((x[t]<1)&(y[t]<2.4))", "blockLow")
    testMonitor.addSubform(
        "((x[t]>0.5)&(y[t]<5))&((x[t]<1)&(y[t]>2.6))", "blockHigh")

    testMonitor.addSubform(
        "(((2*x[t])>8)&((2*y[t])>4))&(((2*x[t])<10)&((2*y[t])<6))", "goal")

    testMonitor.addSubform(
        "((x[t]>4)&(y[t]>2))&((x[t]<5)&(y[t]<3))", "goal2")

    # testMonitor.addSubform(
    #     "((x[t]>4)&(y[t]>2))&((x[t]<5)&(y[t]<3))", "goal")

    testMonitor.add_predicate(
        "(((x[t]-xe[t])*(x[t]-xe[t]))+((y[t]-ye[t])*(y[t]-ye[t])))<"+str(circRadius), "region")

    testMonitor.addSubform("(region)|((blockLow)|(blockHigh))", "collision")

    if formulaStr == 'stayIn.stl':
        testMonitor.setFormula(
            "(F[0,10](G[0,3](region)))&(F[15,20](goal))")
        timeHorz = 10.0
    if formulaStr == 'thinGap.stl':
        testMonitor.setFormula(
            "(G[0,20](!(collision)))&(F[15,20](goal2))")
        timeHorz = 20.0
    if formulaStr == 'thinGapWeighted.stl':
        testMonitor.setFormula(
            "(G[0,20](!(collision)))&(F[15,20](goal))")
        timeHorz = 20.0
    elif formulaStr == 'reachAvoid.stl':
        testMonitor.setFormula(
            "(G[0,60](!(region)))&(F[50,60](goal))")
        timeHorz = 60
    elif formulaStr == 'stayIn2.stl':
        testMonitor.setFormula(
            "G[0,20](region)")
        timeHorz = 20
    elif formulaStr == 'long.stl':
        testMonitor.setFormula(
            # there was a bug in the RoSI tool where the time domain of a predicate nested inside of a nested eventually was incorrect, leading to a non-monotonically decreasing upper bound, which is incorrect and hurt planning performance. region&region forces a conjunction in there to sidestep the bug.
            "(G[0,80](((!(region))|(F[0,10](goal)))&((!(goal))|(F[0,10]((region)&(region))))))&(F[0,5](goal))")
        timeHorz = 90.0

    return testMonitor, timeHorz


class manager:
    """This class monitors the performance of the actual system, and enables/disables
    other nodes as needed
    - It subscribes to the user command and the true state of the system
    - It publishes to the planning topic"""

    def __init__(self):
        #########################
        # initialize parameters #
        #########################
        # stl driver
        self.driver = stlrom.STLDriver()

        self.programRunning = True

        # initialize memory length
        self.memoryLength = 0

        # initialize memory data
        self.memory = np.array([])

        # data to publish
        self.dataToPublish = Float64MultiArray()

        self.floatDataToPublish = Float64()

        # "log time"
        self.logTime = 0

        self.logDt = rospy.get_param("/stlEvaluationDt")

        self.circRadiusSquared = rospy.get_param("/regionRadiusSquared")

        self.last_observation = []

        self.trajLen = 0

        ###############################
        # initialize node information #
        ###############################
        # define topics to publish to
        self.planningTopic = rospy.Publisher(
            'planning', Bool, queue_size=10)

        self.formulaTopic = rospy.Publisher(
            'remainingFormula', String, queue_size=10)

        self.stateObservationsTopic = rospy.Publisher(
            'formSplitStateObservations', Float64MultiArray, queue_size=10)

        self.stateObservationsLogTopic = rospy.Publisher(
            'stateObservations', Float64MultiArray, queue_size=10)

        self.rosiTopic = rospy.Publisher(
            'rosi', Float64MultiArray, queue_size=10)

        self.trajLenTopic = rospy.Publisher(
            'trajLen', Float64, queue_size=10)

        self.distFromCircTopic = rospy.Publisher(
            'distFromCirc', Float64, queue_size=10)

        self.logManagerTime = rospy.Publisher(
            'logManagerTime', Float64, queue_size=10)

        self.logMemoryVal = rospy.Publisher(
            'logMemoryVal', Float64, queue_size=10)

        # initialize node
        rospy.init_node('manager', anonymous=True)

        # define subscriber callbacks
        rospy.Subscriber('userCommand', String, self.receive_user_in)
        rospy.Subscriber('observations', Float64MultiArray,
                         self.observe)

    def receive_user_in(self, data):
        """initialize monitor with given formula, and enable planners"""
        self.testMonitor, self.timeHorz = initialize_driver(
            data.data, self.circRadiusSquared)
        maxMemoryVal = self.testMonitor.getMaxMemory()
        self.memory = np.array([]).reshape([0, 7])
        self.logMemoryVal.publish(data=maxMemoryVal)
        self.memoryLength = int(maxMemoryVal / self.logDt) + 1
        # publish planning
        self.planningTopic.publish(1)
        self.formulaTopic.publish(self.testMonitor.printDriver())
        success = self.driver.parse_string(self.testMonitor.printDriver())
        while (not success):
            success = self.driver.parse_string(self.testMonitor.printDriver())
        # rospy.loginfo("Executing...")

    def observe(self, data):
        """receive and save robot and predicate states"""
        if data.data[0] >= self.logTime:
            obsservation = list(data.data[0:3])
            obsservation.extend(list(data.data[5:]))
            self.testMonitor.update_data(obsservation)
            driverString = self.testMonitor.printSplit()
            if np.shape(self.memory)[0] == self.memoryLength:
                self.memory = np.delete(self.memory, (0), axis=0)
            self.memory = np.vstack((self.memory, np.array(data.data)))
            self.dataToPublish.data = np.reshape(
                self.memory, [np.shape(self.memory)[0]*np.shape(self.memory)[1]]).tolist()
            self.stateObservationsTopic.publish(self.dataToPublish)
            self.stateObservationsLogTopic.publish(data)
            self.formulaTopic.publish(driverString)
            # print("raw data: ")
            # print(data.data)
            # print("observation to add to rosi:")
            # print(obsservation)
            self.driver.add_sample(obsservation)
            robs = self.driver.get_online_rob("phi")
            # print("\nTime: " + str(self.logTime))
            # print("Robustness:")
            # print(robs)
            # print("rcpe formula:")
            # print(driverString)
            # print(robs)
            # driver = stlrom.STLDriver()
            # driver.parse_string(driverString)
            # testData = np.hstack((self.memory[:, 0:3], self.memory[:, 5:]))
            # driver.data = testData
            # robs2 = driver.get_online_rob("phi")
            # flag1 = (robs[0] - robs2[0]) > 1e-2
            # flag2 = (robs[1] - robs2[1]) > 1e-2
            # flag3 = (robs[2] - robs2[2]) > 1e-2
            # if flag1 | flag2 | flag3:
            #     print("\nDIFFERENCE")
            #     print("Actual Robustness:")
            #     print(robs)
            #     print("RCPE Robustness:")
            #     print(robs2)

            self.dataToPublish.data = robs
            self.rosiTopic.publish(self.dataToPublish)
            self.logTime = self.logTime + self.logDt
            self.floatDataToPublish.data = self.logTime
            self.logManagerTime.publish(self.floatDataToPublish)
            if (robs[1] >= 0) | (self.logTime >= self.timeHorz):
                rospy.loginfo("Success!!")
                self.planningTopic.publish(0)
                self.programRunning = False
            if robs[2] < 0:
                rospy.loginfo("Failure :(")
                self.planningTopic.publish(0)
                self.programRunning = False
            newX = obsservation[1]
            newY = obsservation[2]
            newXe = obsservation[3]
            newYe = obsservation[4]
            if len(self.last_observation) > 0:
                oldX = self.last_observation[1]
                oldY = self.last_observation[2]
                dist = math.sqrt(((newX-oldX)*(newX-oldX)) +
                                 ((newY-oldY)*(newY-oldY)))
                self.trajLen = self.trajLen + dist
                self.floatDataToPublish.data = self.trajLen
                self.trajLenTopic.publish(self.floatDataToPublish)
            self.last_observation = obsservation
            distFromCenter = math.sqrt(((newX-newXe)*(newX-newXe)) +
                                       ((newY-newYe)*(newY-newYe)))
            distFromCircle = distFromCenter - math.sqrt(self.circRadiusSquared)
            self.floatDataToPublish.data = distFromCircle
            self.distFromCircTopic.publish(self.floatDataToPublish)

    def run(self):
        """Monitor the system execution"""

        # define rate
        rate = rospy.Rate(50)

        # main loop
        while not rospy.is_shutdown() and self.programRunning:

            rate.sleep()


if __name__ == '__main__':
    try:
        myManager = manager()
        myManager.run()
    except rospy.ROSInterruptException:
        pass
