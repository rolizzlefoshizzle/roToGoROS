import rospy
from std_msgs.msg import Bool
import subprocess
import rospkg
import os

# a lot of help from https://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/


class logger:
    """This class monitors the performance of the actual system, and enables/disables
    other nodes as needed
    - It subscribes to the user command and the true state of the system
    - It publishes to the planning topic"""

    def __init__(self):
        #########################
        # initialize parameters #
        #########################

        self.programRunning = True
        ###############################
        # initialize node information #
        ###############################
        # initialize node
        rospy.init_node('logger', anonymous=True)

        formula = rospy.get_param("/userInFormula")
        benchTest = rospy.get_param("/benchTest")
        formula = formula[:-4]  # remove ".stl" from the end
        rospy.Subscriber('planning', Bool, self.check_planning)
        rospack = rospkg.RosPack()
        path = rospack.get_path('ro-to-go')
        directory = path + "/bagFiles/" + benchTest + "/" + formula
        bagName = rospy.get_param("/bagFileName")

        self.rosbag_proc = subprocess.Popen(
            ["rosbag", "record", "-a", "-O", bagName], stdin=subprocess.PIPE, cwd=directory)

    def terminate_process_and_children(self):
        ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" %
                                      self.rosbag_proc.pid, shell=True, stdout=subprocess.PIPE)
        ps_output = ps_command.stdout.read()
        retcode = ps_command.wait()
        assert retcode == 0, "ps command returned %d" % retcode
        for pid_str in ps_output.split("\n")[:-1]:
            os.kill(int(pid_str), subprocess.signal.SIGINT)

        self.rosbag_proc.terminate()

    def check_planning(self, data):
        if (not data.data):
            self.terminate_process_and_children()
            self.programRunning = False

    def run(self):
        """publish the stl formula to start motion"""

        # define rate
        rate = rospy.Rate(50)

        # main loop
        while not rospy.is_shutdown() and self.programRunning:

            rate.sleep()


if __name__ == '__main__':
    try:
        myLogger = logger()
        myLogger.run()
    except rospy.ROSInterruptException:
        pass
