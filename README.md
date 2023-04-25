Virtual Testbed for High-Cadence Human-Swarm Control
=============================
## Preliminaries
The virtual testbed is meant to be run on Ubuntu 18.04 LTS with ROS Melodic

## Creating a ROS workspace
To create a ROS workspace:
1. Follow the directions at `http://wiki.ros.org/catkin/Tutorials/create_a_workspace` for ROS melodic
2. Clone this repository to the `~/catkin_ws/src` directory of your catkin workspace.

## Additional Packages
You will need to install scipy for python2, the ROS grid_map package (https://github.com/anybotics/grid_map), and the ROS melodic rosbridge to run our testbed.

To install scipy for python2, run:
`pip install scipy`

To install the grid_map package, run:
`sudo apt install ros-melodic-grid-map`

To install ROs melodic rosbridge, run:
`sudo apt install ros-melodic-rosbridge-suite`

If you are missing additional dependencies for the grid_map package, take a look at their README on github for debugging tips: https://github.com/anybotics/grid_map

## Networking Setup
### /etc/hosts
Each player will need to add the other player's hostname and ip address to their /etc/hosts file
The player hosting the game environment will need to add both players' touchscreen devices to their /etc/hosts file as well.

### SSH
The hosting player will need to setup ssh keys. Make sure to use an rsa 4096 bit key instead of ed25519 (ed25519 has problems with ROS).
The following guide by Digital Ocean is a good resource for getting started:
https://www.digitalocean.com/community/tutorials/how-to-set-up-ssh-keys-2

### UFW rules
Both players must be on the same local network. The hosting player will have to add a ufw rule to allow incoming connections from the ip address of the non-hosting player, as well as from the touchscreen devices the hosting player and non-hosting player are using.

For example, if you are the host and the ip address of the other player is 192.168.1.1, add the ufw rule as follows:
`sudo ufw allow from 192.168.1.1`

To delete the ufw rule when you are done using the testbed, run:
`sudo delete allow from 192.168.1.1`

The hosting player will have up to 3 new ufw rules in total. 

The non-hosting player will need to setup ufw rules allowing incoming connections from the host player's PC and ssh connections.
For ssh connections, the ufw can be added as follows:
`sudo ufw allow ssh`

### env.sh
The non-hosting player will have to modify their env.sh file, changing the <host_address> in line 5:
`export ROS_MASTER_URI=<host_address>`
to the host_address of the hosting PC


## Running the Game Environment
To Run the Game Environment:
1. Source the ros environment using `source devel/setup.bash`
- If you are hosting the game, make sure your ROS_MASTER_URI environment variable is set to: http://localhost:11311
  run: `export ROS_MASTER_URI=http://localhost:11311`
- If you are not the host, set your ROS_MASTER_URI to the host computer's ROS_MASTER_URI
- The host will need to run `ifconfig` or `ip address show` to determine their IP address. The host will also need to ensure that port 11311 on their system can receive connections
- If you are the host, you will need to set ROSLAUNCH_SSH_UNKNOWN=1 to allow ssh connections that are not in known_hosts:
  `export ROSLAUNCH_SSH_UNKNOWN=1`

2. Start the ros game environment  by running `roslaunch ergodic_humanswarmcollab_sim tablet.launch num_agents:=a team:=b host:=c other_machine_address:=d other_user:=e`,
where `a`=the total number of agents in the game (i.e a=20 means there will be initially be 20 agents spawned in the environment, divided into two teams of 10 agents, representing the red and blue teams),
`b` denotes the color of the team you are controlling,
`c` denotes whether you are hosting the game or not (enter true/false).
`d` denotes the hostname of the non-hosting player's PC. You only need to enter this if you are the host.
`e` denotes the username of the non-hosting players' PC (for establishing an ssh connection). You only need to enter this if you are the host.
- If you are hosting the game, the tablet.launch file will create the game environment
- If you are not hosting the game, tablet.launch will only open an rviz window through which you can view the state of the game (hosted on the other player's PC).

The non-hosting player needs to run:
`roslaunch ergodic_humanswarmcollab_sim tablet.launch host:=false team:=a`
where `team` is their team's color, which is different from the host player's team color

3. Launching the touch screen interface:

Create a virtual python environment in the touchscreen directory; download the packages from requirements.txt using pip

Activate the environment

Then run:
`python3 ergodic_interface_v12 --team <red/blue> --host <yes/no> --address <ip_address>` where:
- team argument should be `red` or `blue` depending upon which team you would like to control
- host argument should be `yes` or `no` depending upon if you are the player that is hosting the game environment 
- if you are not the host (host=no), the address argument denotes the hostname or ip address of the hosting player's PC on the local network both players are on

Both players must click `connect` on their touch screens in order for the game to start. 

NOTE: in the current version of the testbed environment, the user must specify which team they are on, meaning they will have to work out which team they are on ahead of time with the person they are playing against. Future versions of the testbed environment will automatically assign the team to a player.

## Running a single player training environment
If you want to gain a better understanding of how your swarm responds to touchscreen input without playing against another player, you can run a single player training environment with one team that shows you were your team would capture agents on the opposing team.

To start the single player environment, run:
`roslaunch ergodic_humanswarmcollab_sim tablet.launch single_player:=true team:=a num_agents:=b`
where `a` is the desired color of your team, and `b` is the desired number of agents.

To start the touchscreen, run:
`python3 ergodic_interface_v12 --team <a> --host yes`
where `a` is the same color you entered to launch the single player environment.

Areas where your team would be able to capture members of the opposing team are shown in black

## Flocking Demo
To run the flocking demo:
- start a single player environment with the red team (the red team currently defaults to using a flocking controller)
- launch the touchscreen and click connect to get the red agents moving
- publish to the /flocking_command topic: x can be 0.0->1.0, y can be 0.0->1.0. migration weight should be 50->100.

## Miscellaneous Notes:
- The game environment tends to lag with  30+ agents.

## Copyright and License
The implementations of swarm_testbed contained herein are copyright (C) 2021 - 2022 by Joel Meyer, Allison Pinosky and are distributed under the terms of the GNU General Public License (GPL) version 3 (or later). Please see the LICENSE for more information.

Contact: joelmeyer@u.northwestern.edu

Lab Info: Todd D. Murphey https://murpheylab.github.io/ Northwestern University

