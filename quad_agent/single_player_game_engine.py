import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Pose, Point
from std_msgs.msg import Empty, String, Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from grid_map_msgs.msg import GridMap
import tf
from tf import transformations as trans

from scipy.signal import convolve2d
from scipy.ndimage import generic_filter

from collections import defaultdict ## used for creating a ck dict that has 3 values for each key value
from ergodic_humanswarmcollab_sim.msg import Ck

from d_erg_lib.basis import Basis
from d_erg_lib.utils import convert_ck2dist

from timeit import default_timer as timer ##used to time how long each loop of update_game() takes

class SinglePlayerGameEngine(object):
    def __init__(self, team):
        """ Initializes the game environment in ROS rviz."""
        self._team = team
        rospy.init_node("single_player_game_engine")

        self._rate = rospy.Rate(30)
        self._ck_dict = defaultdict(list)       #format: [agent_name] -> {agent_team, ck, time_ck_was_received}
        rospy.Subscriber('ck_link', Ck, self._ck_link_callback)

        # Used for getting the time-averaged trajectory of both teams; needed for determining whether an agent has been captured in update_rendering()
        grid = np.meshgrid(*[np.linspace(0, 1, 50.) for _ in range(2)])
        self.grid = np.c_[grid[0].ravel(), grid[1].ravel()]
        self.explr_space = np.array([[0., 0.],[1., 1.]], dtype=np.float32)
        self.basis = Basis(self.explr_space, num_basis=10)

        if (self._team == "red"):
            self._ck_pub = rospy.Publisher('/red_ck_plot', Float32MultiArray, queue_size=1)
            self._rviz_pub = rospy.Publisher('/red_capture_area', GridMap, queue_size=1)
        else:
            self._ck_pub = rospy.Publisher('/blue_ck_plot', Float32MultiArray, queue_size=1)
            self._rviz_pub = rospy.Publisher('/blue_capture_area', GridMap, queue_size=1)

        # Sending the time-averaged behavior of each team to Rviz (so we have an idea of who's getting captured)
        self._rviz_msg = GridMap()
        _arr = Float32MultiArray()
        _arr.data = None
        _arr.layout.dim.append(MultiArrayDimension())
        _arr.layout.dim.append(MultiArrayDimension())
        _arr.layout.dim[0].label="column_index"
        _arr.layout.dim[0].size=50
        _arr.layout.dim[0].stride=50*50
        _arr.layout.dim[1].label="row_index"
        _arr.layout.dim[1].size=50
        _arr.layout.dim[1].stride=50
        self._rviz_msg.layers.append("elevation")
        self._rviz_msg.data.append(_arr)
        self._rviz_msg.info.length_x=10
        self._rviz_msg.info.length_y=10
        self._rviz_msg.info.pose.position.x=5
        self._rviz_msg.info.pose.position.y=5
        self._rviz_msg.info.header.frame_id = "world"
        self._rviz_msg.info.resolution = 0.2 # this is linked to the num_points was 0.2 for 10

    def run(self):
        """ Updates the game environment at 30 Hz. """
        while not rospy.is_shutdown():
            self.update_game()
            self._rate.sleep()

    def norm_dist(self, dist):
        """ Shifts all values of the input distributions (which are arrays representing the spatial distributions of each team) 
        to be from 0 -> 1. 
            
        Keyword arguments:
        dist -- the spatial statistics of a team's trajectories averaged over the sampling interval.
        """
        if (np.min(dist) <= 0):
            dist += np.min(dist)
        else:
            dist -= np.min(dist)
        if (np.max(dist) > 0):
            dist /= np.max(dist)
        return dist
            
    def update_game(self):
        """ Updates the state of the game environment at every timestep. Publishes where each team can capture members of the opposing team. """
        #start = timer() #start timer
        if (any("blue" in v for v in self._ck_dict.itervalues()) or any("red" in v for v in self._ck_dict.itervalues())):
            cks = []
            for key in self._ck_dict:
                cks.append(self._ck_dict[key][1])

            _mean = np.mean(cks, axis=0)
            _ck_dist = self.norm_dist(convert_ck2dist(self.basis, _mean, grid=self.grid))
            _ck_plot = np.reshape(_ck_dist,(50,50))

            # get the avg. value over some neighborhood centered at each position in the ck plots
            _kernel_avg = self.kernel_average(_ck_plot)

            # send the blue kernel average to agent.py
            _msg = Float32MultiArray()
            _flat = _kernel_avg.flatten()
            _msg.data = _flat.tolist()
            self._ck_pub.publish(_msg)

            # send the blue kernel average to rviz (shows where the blue team can capture red agents)
            temp = _flat * 10**6             
            self._rviz_msg.data[0].data = temp[::-1]
            self._rviz_pub.publish(self._rviz_msg)

        #end = timer() #end timer
        #rospy.logwarn("update_game() loop took: %f seconds ", (end-start)) #print how long the update_game() loop took
        

    def _ck_link_callback(self, msg):
        """ Updates the dictionaries containing information on each team every time a ck message from one of the agents is received. """
        msg_timestamp = rospy.get_time()
        if self._ck_dict.has_key(msg.name):
            self._ck_dict[msg.name][0] = msg.team
            self._ck_dict[msg.name][1] = np.array(msg.ck, dtype=np.float32)
            self._ck_dict[msg.name][2] = msg_timestamp
        else:
            self._ck_dict.update({msg.name : [msg.team, np.array(msg.ck, dtype=np.float32), msg_timestamp]})

    def kernel_average(self, x):
        """ Calculates the area in which each team can capture members of the opposing team. 
        Each agent exerts a 15x15 capture area around its current position at each timestep. """

        ## 15x15 kernel
        #kernel = np.ones((15, 15), dtype=np.float32)
        #kernel[1:14, 1:14] = 0
        #kernel[2:13, 2:13] = 0

        ## 10x10 kernel
        #kernel = np.ones((10,10), dtype=np.float32)
        #kernel[1:9, 1:9] = 0

        ## 5x5 kernel
        kernel = np.ones((5, 5), dtype=np.float32)
        kernel[1:4, 1:4] = 0
            
        neighbor_sum = convolve2d(
            x, kernel, mode='same',
            boundary='fill', fillvalue=0)

        num_neighbor = convolve2d(
            np.ones(x.shape), kernel, mode='same',
            boundary='fill', fillvalue=0)

        return (neighbor_sum/num_neighbor)
