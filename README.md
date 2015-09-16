# RoboCape-ROS

ROS packages for RoboCape

## Setup procedure

### ROS multiple machines procedure

Replace ra-server and ra-beagle with respective IP addresses or setup correspondance in /etc/hosts

On PC (IP: ra-server):
```bash
export ROS_IP=ra-server
export ROS_HOSTNAME=ra-server
export ROS_MASTER_URI=http://ra-server:11311
roscore
```

On another terminal on PC (IP: ra-server):
```bash
export ROS_IP=ra-server
export ROS_HOSTNAME=ra-server
export ROS_MASTER_URI=http://ra-server:11311
```

On the beaglebone (IP: ra-beagle):
```bash
export ROS_IP=ra-beagle
export ROS_HOSTNAME=ra-beagle
export ROS_MASTER_URI=http://ra-server:11311
```
Then launch your node


### For MATLAB (experimental)

On PC (IP: ra-server) from MATLAB:
```matlab
setenv('ROS_IP','ra-server')
setenv('ROS_HOSTNAME','ra-server')
setenv('ROS_MASTER_URI','http://ra-server:11311')
```
