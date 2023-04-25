#!/usr/bin/env python
import argparse
import rospy

from quad_agent import QuadVisual


parser = argparse.ArgumentParser()
parser.add_argument('num_agents', type=int, help='agent number', default=1)
args, unknown = parser.parse_known_args()

if __name__ == '__main__':
    num_agents = args.num_agents
    agent_names = []
    for i in range(num_agents):
        name = 'agent{}'.format(i)
        agent_names.append(name)
    agent_rendering = QuadVisual(agent_names, num_agents)
    try:
        agent_rendering.run()
    except rospy.ROSInterruptException:
        pass
