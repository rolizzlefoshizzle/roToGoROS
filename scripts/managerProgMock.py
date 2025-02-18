#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Bool, Float64MultiArray, Float64
import stlrom
import time
import formulaProgression

# todo: add cleanup process to allow multiple motion planning runs


def initialize_driver(dim, formulaStr, circRadius):
    testMonitor = formulaProgression.stlManager(
        dim, "signal x, y, xe, ye")
    testMonitor.add_axis_aligned_predicate(0, True, 0, "x[t]>0")
    testMonitor.add_axis_aligned_predicate(0, True, 0.5, "x[t]>0.5")
    testMonitor.add_axis_aligned_predicate(0, True, 4, "x[t]>4")
    testMonitor.add_axis_aligned_predicate(0, True, 4, "(2*x[t])>8")
    testMonitor.add_axis_aligned_predicate(0, False, 5, "x[t]<5")
    testMonitor.add_axis_aligned_predicate(0, False, 5, "(2*x[t])<10")
    testMonitor.add_axis_aligned_predicate(0, False, 1, "x[t]<1")
    testMonitor.add_axis_aligned_predicate(1, True, 0, "y[t]>0")
    testMonitor.add_axis_aligned_predicate(1, True, 4, "y[t]>4")
    testMonitor.add_axis_aligned_predicate(1, True, 2, "y[t]>2")
    testMonitor.add_axis_aligned_predicate(1, True, 2, "(2*y[t])>4")
    testMonitor.add_axis_aligned_predicate(1, True, 2.6, "y[t]>2.6")
    testMonitor.add_axis_aligned_predicate(1, False, 5, "y[t]<5")
    testMonitor.add_axis_aligned_predicate(1, False, 3, "y[t]<3")
    testMonitor.add_axis_aligned_predicate(1, False, 3, "(2*y[t])<6")
    testMonitor.add_axis_aligned_predicate(1, False, 2.4, "y[t]<2.4")
    testMonitor.addSubform(
        "((x[t]>0.5)&(y[t]>0))&((x[t]<1)&(y[t]<2.4))", "blockLow")
    testMonitor.addSubform(
        "((x[t]>0.5)&(y[t]<5))&((x[t]<1)&(y[t]>2.6))", "blockHigh")

    testMonitor.addSubform(
        "(((2*x[t])>8)&((2*y[t])>4))&(((2*x[t])<10)&((2*y[t])<6))", "goal")

    testMonitor.addSubform(
        "((x[t]>4)&(y[t]>2))&((x[t]<5)&(y[t]<3))", "goal2")

    # testMonitor.addSubform(
    #     "(x[t]>4)", "goal")
    #
    # testMonitor.addSubform(
    #     "((x[t]>4)&(y[t]>2))&((x[t]<5)&(y[t]<3))", "goal")

    myCircStr = "(((x[t]-xe[t])*(x[t]-xe[t]))+((y[t]-ye[t])*(y[t]-ye[t])))<" + \
        str(circRadius)
    testMonitor.add_dynamic_circle_predicate(
        0, 2, 1, 3, circRadius, myCircStr)
    testMonitor.addSubform(
        myCircStr, "region")
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
        # testMonitor.setFormula(
        #     "(G[0,10](region))&(F[10,20](goal))")
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

        # data to publish
        self.dataToPublish = Float64MultiArray()

        self.floatDataToPublish = Float64()

        # "log time"
        self.logTime = 0

        self.logDt = rospy.get_param("/stlEvaluationDt")

        self.circRadius = rospy.get_param("/regionRadiusSquared")

        ###############################
        # initialize node information #
        ###############################
        # define topics to publish to
        self.planningTopic = rospy.Publisher(
            'planningMock', Bool, queue_size=10)

        self.formulaTopic = rospy.Publisher(
            'remainingFormulaMock', String, queue_size=10)

        self.stateObservationsLogTopic = rospy.Publisher(
            'stateObservationsMock', Float64MultiArray, queue_size=10)

        # initialize node
        rospy.init_node('manager', anonymous=True)

        # define subscriber callbacks
        rospy.Subscriber('userCommand', String, self.receive_user_in)
        rospy.Subscriber('observations', Float64MultiArray,
                         self.observe)

    def receive_user_in(self, data):
        """initialize monitor with given formula, and enable planners"""
        self.testMonitor, self.timeHorz = initialize_driver(
            4, data.data, self.circRadius)
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
            startTime = time.time()
            self.testMonitor.update_formula(obsservation)
            self.floatDataToPublish.data = time.time() - startTime
            driverString = self.testMonitor.printDriver()
            self.driver.add_sample(obsservation)
            robs = self.driver.get_online_rob("phi")
            if (robs[1] >= 0) | ("true" in driverString):
                rospy.loginfo("Success!!")
                self.planningTopic.publish(0)
                self.programRunning = False
            if robs[2] < 0 | ("false" in driverString):
                rospy.loginfo("Failure :(")
                self.planningTopic.publish(0)
                self.programRunning = False
            self.stateObservationsLogTopic.publish(data)
            self.formulaTopic.publish(driverString)
            self.dataToPublish.data = robs
            self.logTime = self.logTime + self.logDt
            self.floatDataToPublish.data = self.logTime

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
