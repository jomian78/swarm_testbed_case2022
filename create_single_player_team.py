#!/usr/bin/env python

import rospy
import roslaunch
import argparse
from std_srvs.srv import Empty, EmptyResponse

parser = argparse.ArgumentParser()
parser.add_argument('num_agents', type=int, help='agent number', default=1)
parser.add_argument('team', type=str, help='the color of your team', default="red")
args, unknown = parser.parse_known_args()


if __name__ == '__main__':
    package = 'ergodic_humanswarmcollab_sim'

    nodes = []
    processes = []
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    num_agents = args.num_agents
    team = args.team
    if team == "red":
        rospy.init_node("red_team", anonymous=True)
    else:
        rospy.init_node("blue_team", anonymous=True)

    executable = "create_single_player_agent.py"
    
    for i in range(num_agents):
        node_name = "agent{}".format(i)
        args = "{name} {id} {total_agents} {color}".format(name=node_name, id=i, total_agents=num_agents, color=team)

        nodes.append(
            roslaunch.core.Node(package=package, node_type=executable, name=node_name, args=args, output="screen") #TODO: we may want to send the output to a log file instead
        )
                                    
        processes.append(launch.launch(nodes[-1]))

    rospy.spin()
    


