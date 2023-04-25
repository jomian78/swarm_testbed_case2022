#!/usr/bin/env python
import argparse
import rospy

from quad_agent import SinglePlayerQuadVisual


parser = argparse.ArgumentParser()
parser.add_argument("num_agents", type=int, help="agent number", default=1)
parser.add_argument("team", type=str, help="the color of the team you are controlling", default="red")
args, unknown = parser.parse_known_args()

if __name__ == '__main__':
    num_agents = args.num_agents
    team = args.team
    agent_names = []
    for i in range(num_agents):
        name = 'agent{}'.format(i)
        agent_names.append(name)
    agent_rendering = SinglePlayerQuadVisual(agent_names, num_agents, team)
    try:
        agent_rendering.run()
    except rospy.ROSInterruptException:
        pass
