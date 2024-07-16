Step-by-Step Guide for Cloning and Executing the Repository
1. Clone the Repository

git clone https://github.com/SaiVinay023/ros2_ws.git
cd <your-repo-folder>

2. Setup the ROS 2 Workspace

cd ros2_ws

3. Install Dependencies


rosdep update
rosdep install --from-paths src --ignore-src -r -y
If rosdep is not installed, you can install it via:

sudo apt-get install python3-rosdep

4. Build the Workspace

colcon build

5. Source the Workspace
   
source install/setup.bash
7. Run the Instrumented Launch File
   
ros2 launch bt_bumpgoo run_instrumented.launch
Expected Errors and Issues
1. Error: sensor_msgs is not defined

Copy code
NameError: name 'sensor_msgs' is not defined
This indicates a missing import in the monitor.py file. Ensure you have the correct imports:

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

2. Error: Attribute 'exec' not found in Entity 'node'

Caught exception in launch (see debug for traceback): Attribute 'exec' of type '<class 'str'>' not found in Entity 'node'
This means there is an issue with the launch file syntax. Make sure your launch files are in the correct format:


<launch>
    <node pkg="bt_bumpgoo" type="bt_bumpgo_main" name="bt_bumpgo_main" output="screen">
        <param name="use_sim_time" value="true"/>
        <remap from="input_scan" to="/scan_raw"/>
        <remap from="output_vel" to="/cmd_vel"/>
    </node>

    <node pkg="monitor" type="monitor_0" name="monitor_0" output="screen">
        <param name="use_sim_time" value="true"/>
    </node>
</launch>
3. Error: No element found: line 1, column 0
This error suggests an issue with parsing the launch file, indicating the file may be empty or improperly formatted.

Final Steps
Rebuild the workspace:

colcon build
In a terminal we do:

 cd ~/ros2_ws/
 . install/setup.bash
 ros2 launch src/monitor/launch/monitor.launch
 
Then, in another terminal we do:

$ cd ~/ros2_ws/
$ . install/setup.bash
$ ros2 launch src/bt_bumpgoo/launch/run_instrumented.launch

ros2 launch bt_bumpgoo run_instrumented.launch
Note for the Professor
The repository contains a ROS 2 workspace with the bt_bumpgoo package and monitor package for ROSMonitoring. The above steps should help in setting up the environment. 
