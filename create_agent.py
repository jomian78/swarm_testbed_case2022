#!/usr/bin/env python

import os
import argparse
import rospy

from quad_agent import Agent


parser = argparse.ArgumentParser()
parser.add_argument('agent_name', type=str, help='agent name', default="thaddius")
parser.add_argument('id_number', type=int, help='id_number', default=0)
parser.add_argument('num_agents', type=int, help='total number of agents', default=1)
args, unknown = parser.parse_known_args()

if __name__ == '__main__':

    agent = Agent(args.agent_name, args.id_number, args.num_agents)
    try:
        agent.run()
    except rospy.ROSInterruptException:
        pass
