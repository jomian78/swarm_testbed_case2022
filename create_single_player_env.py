#!/usr/bin/env python

import os
import argparse
import rospy

from quad_agent import SinglePlayerGameEngine

parser = argparse.ArgumentParser()
parser.add_argument('team', type=str, help='what color team are we controlling', default="red")
args, unknown = parser.parse_known_args()

if __name__ == '__main__':
    single_player_game_engine = SinglePlayerGameEngine(args.team)
    try:
        single_player_game_engine.run()
    except rospy.ROSInterruptException:
        pass
