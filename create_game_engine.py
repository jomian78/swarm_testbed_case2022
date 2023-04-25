#!/usr/bin/env python

import os
import argparse
import rospy

from quad_agent import GameEngine

if __name__ == '__main__':
    game_engine = GameEngine()
    try:
        game_engine.run()
    except rospy.ROSInterruptException:
        pass
