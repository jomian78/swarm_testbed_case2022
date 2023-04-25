import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Pose, Point
from std_msgs.msg import Empty, String, Float32MultiArray
import tf
from tf import transformations as trans
# from tanvas_comms.msg import input_array

from scipy.signal import convolve2d
from scipy.ndimage import generic_filter

from collections import defaultdict ## used for creating a ck dict that has 3 values for each key value
from ergodic_humanswarmcollab_sim.msg import Ck

from d_erg_lib.basis import Basis
from d_erg_lib.utils import convert_ck2dist

import time
import pandas as pd
import os

class QuadVisual(object):
    """
    Renders agents and paths in RVIZ. Agent locations are based only on the
    "current step" and paths only represent previously visited locations
    (no forward projection of planned path). A single instance of this node may
    be used to render the entire swarm (even if nodes update asynchronously).

    Listens to tf.TransformListener to get agent locations from ROS
    (which are broadcasted by agent.py).
    """
    def __init__(self, agent_names, num_agents, scale=0.1):

        rospy.init_node("agent_rendering")

        self._agent_names    = agent_names
        self._num_agents     = num_agents
        self._scale          = scale

        self._agent_markers = MarkerArray()
        self._markers = [
            Marker() for i in range(len(agent_names))
        ]
        self._path_markers = [
            Marker() for i in range(len(agent_names))
        ]
        self._agent_markers.markers = self._markers + self._path_markers

        # Instantiate the publishers
        self._marker_pub = rospy.Publisher('agent/visual', MarkerArray, queue_size=1)

        self.__build_rendering()
        self.listener = tf.TransformListener()
        self._rate = rospy.Rate(30)

        self.path_marker_index = 0

        # Instantiate a dictionary and subscriber that keep track of the team state of all agents
        self.team_dict = {}
        for count, agent in enumerate(agent_names):
            if count < (self._num_agents/2):
                self.team_dict[agent] = "red"
            else:
                self.team_dict[agent] = "blue"

        self.red_kernel_avg = None
        self.blue_kernel_avg = None
        rospy.Subscriber('/blue_ck_plot', Float32MultiArray, self.blue_ck_callback)
        rospy.Subscriber('/red_ck_plot', Float32MultiArray, self.red_ck_callback)

        self.team_pub = rospy.Publisher('/agent_team_status', String, queue_size=1)

        # save how many agents are on each team over time
        self.start_time = time.time()
        self.df_agents_team = pd.DataFrame([[0.0, self._num_agents/2, self._num_agents/2]],columns=["time", "red_agents", "blue_agents"])
        self.save_dir = os.path.dirname(os.path.abspath(__file__))+"/../"
        self.filename = "{}_df_agents_team.json".format(self.start_time)
        self.team_time_counter = 0

    def get_agents_on_each_team(self):
        curr_time = time.time() - self.start_time
        num_red_agents = sum(1 for v in self.team_dict.values() if v == "red")
        num_blue_agents = sum(1 for v in self.team_dict.values() if v == "blue")

        new_df = pd.DataFrame([[curr_time, num_red_agents, num_blue_agents]], columns=["time", "red_agents", "blue_agents"])
        self.df_agents_team = self.df_agents_team.append(new_df, ignore_index=True)

        if (self.team_time_counter % 30) == 0:
            self.df_agents_team.to_json("{}/{}".format(self.save_dir, self.filename))
        self.team_time_counter = self.team_time_counter + 1

    def run(self):
        while not rospy.is_shutdown():
            self.update_rendering()
            self.get_agents_on_each_team()
            self._rate.sleep()

    # def tdist_callback(self,data):
    #     for agent_name, line_m in zip(self._agent_names, self._path_markers):
    #         del line_m.points[:]

    # def reset_callback(self,data):
    #     """ Reset the Rviz path lines generated by the agents. """
    #     for agent_name, line_m in zip(self._agent_names, self._path_markers):
    #         del line_m.points[:]

    def blue_ck_callback(self, msg):
        """ Receive the array denoting where the blue team will
        capture red agents in the testbed environment.
        """
        self.blue_kernel_avg = np.reshape(np.asarray(msg.data), (50,50))

    def red_ck_callback(self, msg):
        """ Receive the array denoting where the red team will
        capture blue agents in the testbed environment.
        """
        self.red_kernel_avg = np.reshape(np.asarray(msg.data), (50,50))

    def update_rendering(self):
        """ At each timestep, add markers (in rviz) to the locations of each agent,
        If an agent enters into an area where it will be captured, switch the
        agent's team color and publish the new team it is on to /agent_team_status.
        """
        if (any("red" in v for v in self.team_dict.values()) and any("blue" in v for v in self.team_dict.values())):
            #for agent_name, marker, line_m in zip(self._agent_names, self._markers, self._path_markers):
            for agent_name, marker in zip(self._agent_names, self._markers):
                try:
                    (trans, rot) = self.listener.lookupTransform(
                        "world", agent_name, rospy.Time(0)
                    )
                    marker.pose.position.x = trans[0]
                    marker.pose.position.y = trans[1]
                    marker.pose.orientation.x = rot[0]
                    marker.pose.orientation.y = rot[1]
                    marker.pose.orientation.z = rot[2]
                    marker.pose.orientation.w = rot[3]

                    # line_m.points.append(
                    #     Point(marker.pose.position.x, marker.pose.position.y,
                    #           0.1))

                    x_index = min(49,int(marker.pose.position.x/10.0 * 49))
                    y_index = min(49,int(marker.pose.position.y/10.0 * 49))

                    #if (self.red_kernel_avg is not None and self.team_dict[agent_name] == "blue" and (self.red_kernel_avg[x_index, y_index] > 0.75*self.red_kernel_avg.max())):
                    if (self.red_kernel_avg is not None and self.team_dict[agent_name] == "blue" and (self.red_kernel_avg[x_index, y_index] >= 0.75)):
                        #rospy.logwarn("%s changed teams from blue to red", agent_name)
                        self.team_dict[agent_name] = "red"
                        marker.color.r = 128.
                        marker.color.g = 0.
                        marker.color.b = 0.
                        team_msg = String()
                        team_msg.data = "%s %s" % (agent_name, self.team_dict[agent_name])
                        self.team_pub.publish(team_msg)
                    #elif (self.blue_kernel_avg is not None and self.team_dict[agent_name] == "red" and (self.blue_kernel_avg[x_index, y_index] > 0.75*self.blue_kernel_avg.max())):
                    elif (self.blue_kernel_avg is not None and self.team_dict[agent_name] == "red" and (self.blue_kernel_avg[x_index, y_index] >= 0.75)):
                        #rospy.logwarn("%s changed teams from red to blue", agent_name)
                        self.team_dict[agent_name] = "blue"
                        marker.color.r = 0.
                        marker.color.g = 96.
                        marker.color.b = 295.
                        team_msg = String()
                        team_msg.data = "%s %s" % (agent_name, self.team_dict[agent_name])
                        self.team_pub.publish(team_msg)
                    else:
                        pass

                    # only display the last 300 marker paths -- this gives the agents visual "tails" that let the user know where the prior positions that are still taking effect are
                    # if (self.path_marker_index < 300):
                    #     self.path_marker_index = self.path_marker_index + 1
                    # else:
                    #     line_m.points.pop(0)

                    #uncomment the next line if you don't want to display any marker paths
                    #line_m.points.pop(0)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

        self._marker_pub.publish(self._agent_markers)

    def __build_rendering(self):
        """Initialize the Rviz environment and renderings of the agents at their starting positions .
        """
        i=0
        for name, agent_marker, path_marker in zip(self._agent_names, self._markers, self._path_markers):
            quad_scale = 0.5 # increased size for grid
            agent_marker.header.frame_id = "world"
            agent_marker.header.stamp = rospy.Time(0)
            agent_marker.ns = name
            agent_marker.id = 0
            agent_marker.action = Marker.ADD
            agent_marker.scale.x = quad_scale
            agent_marker.scale.y = quad_scale
            agent_marker.scale.z = quad_scale
            agent_marker.color.a = 1.0

            if (i < (self._num_agents/2)):
                #red agents
                agent_marker.color.r = 128
                agent_marker.color.g = 0
                agent_marker.color.b = 0
            else:
                #blue agents
                agent_marker.color.r = 0.
                agent_marker.color.g = 96.
                agent_marker.color.b = 295.

            # static spawn agent position (these don't move) -- putting them far off the map for now
            agent_marker.pose.position.x = np.random.uniform(-1000.0, -1000.0)
            agent_marker.pose.position.y = np.random.uniform(-1000.0, -1000.0)
            agent_marker.pose.position.z = np.random.uniform(1.6,4)

            agent_marker.type = Marker.MESH_RESOURCE
            agent_marker.mesh_resource = "package://ergodic_humanswarmcollab_sim/mesh/quad_base.stl"

            # Make Trajectory Lines
            line_scale = 0.1
            path_marker.header.frame_id = "world"
            path_marker.header.stamp = rospy.Time(0)
            path_marker.ns = name+'/path'
            path_marker.id = i
            path_marker.action = Marker.ADD
            path_marker.scale.x = line_scale
            path_marker.color = agent_marker.color
            path_marker.color.a = 0.7
            path_marker.pose.position.z = 0.1
            path_marker.type = Marker.LINE_STRIP
            path_marker.pose.orientation = agent_marker.pose.orientation # added to remove error messages in RVIZ

            i+=1
