# amiro_potential_field_navigation

## References

* ROS Potential Fields Map https://www.youtube.com/watch?v=BDzufMACM2M


## How To: 
### Setup
* open a terminal with ```win+t``` or ```win+r``` and type ```terminal```
* opend the ```.bashrc``` with an editor (```gedit ~/.bashrc```) and append the following:
```
# Robocup RSB/RST/RSC/Proto
F='/vol/robocup/nightly-2017-03-30'
ls $F >/dev/null
export PATH=$F/bin:$F/sbin:${PATH}
export MANPATH=$F/share/man:${MANPATH}
export PKG_CONFIG_PATH=$F/lib/pkgconfig:$F/lib64/pkgconfig:${PKG_CONFIG_PATH}
export LD_LIBRARY_PATH=$F/lib:$F/lib64:$F/usr/lib:${LD_LIBRARY_PATH}
export LIBRARY_PATH=$F/lib:$F/lib64:$F/usr/lib:${LIBRARY_PATH}
export CMAKE_PREFIX_PATH=$F:${CMAKE_PREFIX_PATH}
```
* save it and close the editor.
* type ```source ~/.bashrc``` to commit the changes
* then we need to create a catkin workspace:
* ```source /opt/ros/kinetic/setup.bash && cd && mkdir -p catkin_ws/src && cd catkin_ws/src && catkin_init_workspace```
* then we need to clone the repository:
```cd src && git clone https://github.com/tik0/amiro_potential_field_navigation```
* then we have to clone the submodules:
```cd amiro_potential_field_navigation && git submodule update --init --recursive```

### Build
* open a terminal
* source ros enviroments: ```source /opt/ros/kinetic/setup.bash```
* go into your catkin_ws: ```cd ~/catkin_ws```
* to build the files we need catkin: ```catkin_make```
* after this there will be a ```build``` and a ```devel``` folder. Instead of sourcing the ros enviroment you can source a setup.bash in your devel folder in the catkin_ws like: ```source devel/setup.bash```

### Start
* open a terminal
* source the ros enviroment: ```source devel/setup.bash```
* Copy the file ```rsb.conf``` in your ros-home (~/.ros/): ```cp ~/catkin_ws/src/amiro_potential_field_navigation/potential_field_navigation /rsb.conf ~/.ros/```
* now you can start the launch scripts: ```roslaunch potential_field_navigation start.launch```

### AMiRo Act Tool
* open a terminal
* connect to the amiro with ```ssh root@<amiroIp>```
* Enter the password ```amiropower```
* navigate to the act tool folder ```cd ros_navigation_stack_sense_act_tools```
* start the motorcontrol tool: ```./motorControl -i /amiro<Id>/motor/```