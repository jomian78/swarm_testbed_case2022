#!/usr/bin/env python
# lab 4,  Allison Pinosky, Due 12/7/2020
# Based on paper: "Reynolds flocking in reality with fixed-wing robots: communication range vs. maximum turning rate."
# NOTE: graded for flock of 20 robots with good heading agreement staring from random positions / orientations

def usr(robot):
	import struct
	import math
	import timeit

	""" setup """
	# set led color (to make robots easier to see)
	robot.set_led(0,100,0)  		# green
	# initialize empty vectors for implementations
	migr_point = [0,0] 			# x,y location of migration point
	v_align    = [0,0]   			# alignment vector
	v_coh      = [0,0]			# cohesion vector pointing to COM of all robots
	v_sep      = [0,0] 			# separation vector
	v_migr     = [0,0]			# migration vector
	pose       = [0,0,0] 			# placeholder in case a neighbor message is received before valid pose is received
	# initialize weights and thresholds
	weight_align = 1.05	 		# 1 from paper, may need to change
	weight_coh   = 1. 			# 1 from paper, may need to change
	weight_sep   = 1.15			# 1.2 from paper, may need to change
	weight_migr  = 0.24	  		# 1/500 from paper, but our area is much smaller so had to tune this
	thresh_comm  = 3.0 			# limit communication distance to those within threshold
	thresh_sep   = 0.8 			# limit separation distance (repelling robots) to those within (closer) threshold
	thresh_turn  = 0.5			# turn when angle from combined vector exceeds threshold (radians)
	# initialize empty vectors to keep track of other robots
	swarm_id_list         = []		# robot id
	swarm_vec_list        = []   		# robot location (x,y)
	swarm_angle_list      = []		# angle calculated to robot
	swarm_angle_diff_list = []		# difference between calculated angle and angle received from other robot

	while True:
		""" get current location and calculate distance to migration point to calculate attractive vector """
		pose_t=robot.get_pose() # gets robot's global pose (x, y, theta).
		if pose_t: # check pose is valid before using
			pose=pose_t
			robot.send_msg(struct.pack('iffi', 0, pose[0], pose[1],robot.id)) # send pose x,y in message
			# compute distance to migration point
			dx_migration = migr_point[0]-pose[0] 				# change in global x direction
			dy_migration = migr_point[1]-pose[1] 				# change in global y direction
			angle_migration = math.atan2(dy_migration,dx_migration)-pose[2] # change in theta
			distance = math.sqrt(dx_migration**2 + dy_migration**2)  	# distance to migration point
			## compute migration vector (weighted by distance)
			v_migr = [0,0]
			v_migr[0] = math.cos(angle_migration)*distance*weight_migr
			v_migr[1] = math.sin(angle_migration)*distance*weight_migr

		""" get messages from any neighbors to calculate other vectors (two types: 0=pose, 1=angle) """
		msgs = robot.recv_msg()
		if len(msgs) > 0: # check if message is valid before using
			type_rxed = struct.unpack('i', msgs[0][:4]) # get message type
			# location message
			if type_rxed[0] == 0:
				pose_rxed= struct.unpack('ffi', msgs[0][4:16]) 	# get message xy, id
				# compute distance to robot in message
				dx = pose_rxed[0] - pose[0] 			  	# change in global x direction
				dy = pose_rxed[1] - pose[1] 			  	# change in global y direction
				angle_robot = math.atan2(dy,dx) + pose[2]		# change in theta
				dist = math.sqrt(dx**2 + dy**2)				# distance to robot
				id_robot = pose_rxed[2]					# id of robot sending message
				robot.send_msg(struct.pack('iiif', 1, id_robot, robot.id, angle_robot)) # send angle to robot in message
				# check if robot is in range
				if dist < thresh_comm:
					# if so, check if robot id is in list
					if id_robot in swarm_id_list:
						rxed_idx = swarm_id_list.index(id_robot)
						# if so, update lists
						swarm_vec_list[rxed_idx] = [pose_rxed[0],pose_rxed[1]] # to calculate cohesion and separation vectors
						swarm_angle_list[rxed_idx] = angle_robot # to calculate alignment difference with other message type
					else:
						# if not, add to lists
						swarm_id_list.append(id_robot)
						swarm_vec_list.append([pose_rxed[0],pose_rxed[1]])
						swarm_angle_list.append(angle_robot)
						swarm_angle_diff_list.append(0) # initialize
				else:
					# if too far away, check if id is in list
					if id_robot in swarm_id_list:
						rxed_idx = swarm_id_list.index(id_robot)
						# if so, remove from lists
						swarm_id_list.pop(rxed_idx)
						swarm_vec_list.pop(rxed_idx)
						swarm_angle_list.pop(rxed_idx)
						swarm_angle_diff_list.pop(rxed_idx)
			# angle message
			elif type_rxed[0] == 1:
				angle_rxed= struct.unpack('iif', msgs[0][4:16]) # get message robot_id_to, robot_id_from, angle
				if angle_rxed[0] == robot.id: # only use message if it's sent to this robot
					id_robot = angle_rxed[1]
					# check if angle to robot saved in list
					if id_robot in swarm_id_list:
						# if so, get angle and compare to angle from message
						rxed_idx = swarm_id_list.index(id_robot)
						saved_angle = swarm_angle_list[rxed_idx]
						angle_diff = angle_rxed[2]-math.pi-saved_angle # should differ by pi when aligned
						# angle wrapping from -pi to +pi
						if angle_diff > math.pi:
							angle_diff -= 2*math.pi
						elif angle_diff < -math.pi:
							angle_diff += 2*math.pi
						# save difference to list to compute alignment vector
						swarm_angle_diff_list[rxed_idx] = angle_diff

			# if neighbors stored in list, update vectors (only need to update if message received)
			if len(swarm_id_list) > 0:
				# reset vectors
				v_align = [0,0]
				v_coh 	= [0,0]
				v_sep 	= [0,0]

				## compute alignment vector
				avg_angle = 0
				num_vecs = 0
				# compute mean angle (equal weighting between neighboring robots)
				for i in swarm_angle_diff_list:
					if abs(i) > 0: # count non-zero vectors only (received angle message from other robot)
						num_vecs += 1
					avg_angle += i # sum all angles
				if num_vecs > 0: # error handling
					# divide by number to get mean angle
					avg_angle /= num_vecs
					# use angle to calculate vector and weight
					v_align[0] = math.cos(avg_angle)*weight_align
					v_align[1] = math.sin(avg_angle)*weight_align

				## compute cohesion vector
				# get center of mass (COM) of neighboring robots (swarm_list)
				COM = [0,0]
				num_neighbors = len(swarm_id_list)
				# sum vectors from running list
				for i in swarm_vec_list:
					COM[0] += i[0]
					COM[1] += i[1]
				# divide by number to get mean position
				COM[0] /= num_neighbors
				COM[1] /= num_neighbors
				# get vector from robot to COM
				dx = COM[0] - pose[0] 			  		# change in global x direction
				dy = COM[1] - pose[1] 			  		# change in global y direction
				angle_temp = math.atan2(dy,dx) - pose[2] 		# change in theta (convert from global to local coords)
				# use angle to calculate vector and weight
				v_coh[0] = math.cos(angle_temp)*weight_coh
				v_coh[1] = math.sin(angle_temp)*weight_coh

				## compute separation vector
				for i in swarm_vec_list:
					dx = pose[0] - i[0] 			  	# change in global x direction
					dy = pose[1] - i[1] 			  	# change in global y direction
					angle_temp = math.atan2(dy,dx) - pose[2] 	# change in theta (convert from global to local coords)
					dist = math.sqrt(dx**2 + dy**2)			# distance from current robot to swarm robot
					# sum vectors from running list scaled by inverse of distance
					if dist < thresh_sep:
						v_sep[0] += math.cos(angle_temp)/dist
						v_sep[1] += math.sin(angle_temp)/dist
				# normalize
				v_norm = math.sqrt(v_sep[0]**2 + v_sep[1]**2)
				if v_norm > 0: # error handling
					v_sep[0] /= v_norm
					v_sep[1] /= v_norm
				# weight
				v_sep[0] *= weight_sep
				v_sep[1] *= weight_sep

		""" move based on combined vector angle """
		## combine vectors and calculate angle
		v_combo = [v_align[0] + v_coh[0] + v_sep[0] + v_migr[0], 	# x
				v_align[1] + v_coh[1] + v_sep[1] + v_migr[1]] 		# y
		angle_combo = math.atan2(v_combo[1],v_combo[0]) 	  		# change in theta

		# note: not allowed to go "backward", only turn or go forward
		v_norm = math.sqrt(v_combo[0]**2 + v_combo[1]**2)
		v_mag = 50
		# increase according to magnitude of vector (min = 25)
		if v_norm > 0.5:
			v_mag *= v_norm
		# clip at max velocity
		if v_norm > 2:
			v_mag = 100
		# turn if angle exceeds threshold
		if angle_combo > thresh_turn:
			robot.set_vel(-v_mag,v_mag) 	# rotate counter clockwise
		elif angle_combo < -thresh_turn:
			robot.set_vel(v_mag,-v_mag) 	# rotate clockwise
		# go straight (regardless if turned first)
		robot.set_vel(v_mag,v_mag)
