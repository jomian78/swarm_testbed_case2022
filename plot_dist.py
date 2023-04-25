#!/usr/bin/env python

import os
import argparse
import rospy

from d_erg_lib import DErgPlot

parser = argparse.ArgumentParser()
parser.add_argument('agent_name', type=str, help='agent name', default="thaddius")
parser.add_argument('id_number', type=int, help='id_number', default=0)
args, unknown = parser.parse_known_args()

if __name__ == '__main__':

    plot = DErgPlot(args.agent_name, args.id_number)
    try:
        plot.run()
    except rospy.ROSInterruptException:
        pass
