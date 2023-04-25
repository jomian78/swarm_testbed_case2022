#!/usr/bin/env python

import rospy
import roslaunch
import argparse
from std_srvs.srv import Empty, EmptyResponse

parser = argparse.ArgumentParser()
parser.add_argument('num_agents', type=int, help='agent number', default=1)
args, unknown = parser.parse_known_args()


if __name__ == '__main__':
    package = 'ergodic_humanswarmcollab_sim'

    nodes = []
    processes = []
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    rospy.init_node('red_team', anonymous=True)

    executable = 'create_agent.py'
    num_agents = args.num_agents
    
    # if team==red, start agent index i at 0; else if team==blue, start agent index i at num_agents/2
    for i in range((num_agents/2)):
        node_name = 'agent{}'.format(i)
        args = '{name} {id} {total_agents}'.format(name=node_name, id=i, total_agents=num_agents)

        nodes.append(
            roslaunch.core.Node(package=package, node_type=executable, name=node_name, args=args, output="screen") #TODO: we may want to send the output to a log file instead
        )
                                    
        processes.append(launch.launch(nodes[-1]))

    rospy.spin()
    


