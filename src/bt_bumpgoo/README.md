# Bump-and-Go-with-Behavior-Trees

This practice aims to make the Tiago robot move forward in the bookstore scenario, avoiding the obstacles in its path. When the laser sensor discovers an obstacle nearby in front of the robot, it will reverse, turn, and move forward again until another obstacle is found in its path.

In this practice, the student must:
* Complete the behavior tree nodes and test them separately.
* Create a behavior tree that makes the robot perform the desired behavior.

## Developing the behavior tree nodes

There are 4 BT nodes:
1. Forward. It is an action node that makes the robot move forward constantly. Movement nodes should publish `geometry_msgs/msg/Twist` in the robot velocity topic.
2. Back.  It is an action node that makes the robot move back for 3 seconds.
3. Turn.  It is an action node that makes the robot turn for 3 seconds.
4. IsObstacle. It is a condition node that checks for an obstacle in front of the robot. Subscribe to the laser topic containing `sensor_msgs/msg/LaserScan` messages and read the central reading for distances.

Complete the code where you find `// Complete here`

After that, tree tests like `behavior_tree_xml/test_turn` will be developed to test the BT nodes separately. If you want, you can use Groot, the behavior tree editor, for this (`ros2 run Groot Groot`). 
![Captura desde 2024-06-02 17-46-29](https://github.com/SIGSOFT-Summer-Winter-School/Bump-and-Go-with-Behavior-Trees/assets/3810011/bcbb613f-fb22-4383-86ab-bb3b18a4bb9e)

To test them, select which one to run in the main program `bt_bumpgo_main.cpp`.


