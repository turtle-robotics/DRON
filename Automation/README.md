# Automation subteam readme

### this is a temporary holding place for the code that the automation team is working on, eventually we will merge this with the ws_sensor_combined folder but first i will have to go in and remove every submodule so we can actually push code in there.

the two offboard.cpp files go into this directory ws_sensor_combined/src/px4_ros_com/src/examples/offboard/
the cmake file goes into ws_sensor_combined/src/px4_ros_com/

---

### helpful commands
use this command in a separate  terminal to start the client-agent bridge, this allows PX4 to communicate with our ROS2 nodes, the drone will not connect with ROS if this command is not running

- `MicroXRCEAgent udp4 -p 8888`

---

To start multiple simulations you need to run the following commands in seperate terminals 

 - `PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1`
 - `PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 0`

the PX4_GZ_STANDALONE=1 flag tells PX4 that this is not a separate instance of the simulation, this allows multiple drones to be placed within the same simulation, it defaults to =0 if not specified. 

the PX4_GZ_MODEL_POSE="0,1" determines the x,y starting coordinates of the drone, this defaults to 0,0 when not specified. 

the -i # indicates the ID of the drone, in real life the first drone that is connected has an ID of 0, the second has an ID of 1, and so on. 

you MUST start the simulations with PX4_GZ_STANDALONE=1 before your "root" simulation (the one with PX4_GZ_STANDALONE=0).

---

to build your code run `colcon build`, if it says it cannot find px4_ros_com then run `install/setup.bash` and build again. if you add a new file or dependency to an existing file, you must add that information into the CMake file otherwise it will not be able to build

---

to run your ROS2 node, first build using `colcon build` and then `ros2 run px4_ros_com <name_of_node>`

---

when debugging you should run `ros2 topic list` to see all topics that ROS can currently see, note that you must actually have a simulation running in order to receive topics

---

when debugging with topics or if you dont need to see the GUI you can add the `HEADLESS=1` flag to your simulation, this starts the sim without the GUI which makes it much faster.


### link to my notes
https://docs.google.com/document/d/1eF7B05nmmSWAneBnN3lDzX5psfL4cWMjbJJPyKqC5C8/edit?usp=sharing

