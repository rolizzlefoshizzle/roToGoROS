#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Bool


class initiator:
    """This class monitors the performance of the actual system, and enables/disables
    other nodes as needed
    - It subscribes to the user command and the true state of the system
    - It publishes to the planning topic"""

    def __init__(self):
        #########################
        # initialize parameters #
        #########################

        self.userIn = rospy.get_param("/userInFormula")
        self.programRunning = True
        ###############################
        # initialize node information #
        ###############################
        # define topics to publish to
        self.formulaTopic = rospy.Publisher(
            'userCommand', String, queue_size=10)

        # initialize node
        rospy.init_node('initiator', anonymous=True)

        rospy.Subscriber('planning', Bool, self.check_planning)

    def check_planning(self, data):
        if (not data.data):
            self.programRunning = False

    def run(self):
        """publish the stl formula to start motion"""

        # define rate
        rate = rospy.Rate(50)

        notPublished = True

        startTime = rospy.get_time()

        # main loop
        while not rospy.is_shutdown() and self.programRunning:

            time = rospy.get_time()-startTime

            if (notPublished) & (time > 5):
                self.formulaTopic.publish(self.userIn)
                notPublished = False

            rate.sleep()


if __name__ == '__main__':
    try:
        myInitiator = initiator()
        myInitiator.run()
    except rospy.ROSInterruptException:
        pass
