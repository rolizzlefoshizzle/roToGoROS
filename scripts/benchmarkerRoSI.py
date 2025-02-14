#!/usr/bin/env python
import roslaunch
import rospy
import rospkg
import yaml


# source: https://stackoverflow.com/questions/56140413/automatically-terminate-all-nodes-after-calling-roslaunch


class ProcessListener(roslaunch.pmon.ProcessListener):
    global process_generate_running

    def process_died(self, name, exit_code):
        global process_generate_running
        process_generate_running = False
        rospy.logwarn("%s died with code %s", name, exit_code)


def init_launch(launchfile, process_listener):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(
        uuid,
        [launchfile],
        process_listeners=[process_listener],
    )
    return launch


numRuns = 5
formula = 'stayIn.stl'
method = "formSplitRob"
viaPoints = [1, 2, 3, 4, 5]
horzLen = [5, 10, 15, 20]

rospack = rospkg.RosPack()
path = rospack.get_path('ro-to-go')

rospy.init_node("benchmarker")
for i in range(numRuns):
    for points in viaPoints:
        for horz in horzLen:
            process_generate_running = True
            with open(path + "/config/params.yaml") as f:
                y = yaml.safe_load(f)
                y['userInFormula'] = formula
                y['benchTest'] = "finiteHorz"
                y['localViaPoints'] = points
                y['localTmax'] = horz
                y['bagFileName'] = "via"+str(points) + \
                    "horz"+str(horz)+"run"+str(i)+".bag"
                print(yaml.dump(y, default_flow_style=False, sort_keys=False))
            with open(path + "/config/params.yaml", "w") as ostream:
                yaml.dump(y, ostream, default_flow_style=False,
                          sort_keys=False)
            launch_file = path + "/launch/" + method + ".launch"
            launch = init_launch(launch_file, ProcessListener())
            launch.start()
            # child = subprocess.Popen(["rosbag", "record", "-a"])

            while process_generate_running:
                rospy.sleep(0.05)

            launch.shutdown()

with open(path + "/config/params.yaml") as f:
    y = yaml.safe_load(f)
    y['userInFormula'] = formula
    y['benchTest'] = "random"
    y['localViaPoints'] = 1
    y['localTmax'] = 5
    y['bagFileName'] = "generic.bag"
    print(yaml.dump(y, default_flow_style=False, sort_keys=False))
with open(path + "/config/params.yaml", "w") as ostream:
    yaml.dump(y, ostream, default_flow_style=False, sort_keys=False)
