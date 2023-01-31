# About launch file

Go to the gazebo_ros folder (`/opt/ros/galactic/share/gazebo_ros/launch`) and refer to `spawn_entity_demo.launch.py` file.

```
def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gazebo.launch.py']),
             )

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
                        arguments=['-entity', 'demo', '-database', 'double_pendulum_with_base'],
                        output='screen')

    return LaunchDescription([
        gazebo,

        spawn_entity,
    ])
```
When you copy this file into your package, change `ThisLaunchFileDir()` to `get_package_share_directory("soonduck_simulation")`. Don't forget to import `get_package_share_directory` function.
```
from ament_index_python.packages import get_package_share_directory
```
The code below is the definition of `Node` class (which is used in `launch` files). As you can see below, there is a parameter `executable`  instead of `node_executable`. Therefore, the argument name in `spawn_entity_demo.launch.py` file must be changed to `executable`. Check out the definition of Node.

```
class Node(ExecuteProcess):
    """Action that executes a ROS node."""

    UNSPECIFIED_NODE_NAME = '<node_name_unspecified>'
    UNSPECIFIED_NODE_NAMESPACE = '<node_namespace_unspecified>'

    def __init__(
        self, *,
        executable: SomeSubstitutionsType,
        package: Optional[SomeSubstitutionsType] = None,
        name: Optional[SomeSubstitutionsType] = None,
        namespace: Optional[SomeSubstitutionsType] = None,
        exec_name: Optional[SomeSubstitutionsType] = None,
        parameters: Optional[SomeParameters] = None,
        remappings: Optional[SomeRemapRules] = None,
        arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        **kwargs
    ) -> None:
```
To use your `URDF` file instead of pre-built model in gazebo database, change `spawn_entity` as shown below:
```
spawn_entity = Node(
    package="gazebo_ros",
    executable="spawn_entity.py",
    arguments=["-entity", "soonduck", "-topic", "robot_description"],
    output="screen",
)
```
Here, `spawn_entity` node subscribes `/robot_description` topic where the `URDF` file is published. So you have to parse your `URDF` or `Xacro` file and publish it using `robot_state_publisher`.
```
use_sim_time = LaunchConfiguration("use_sim_time", default="false")

xacro_file = os.path.join(
    get_package_share_directory("soonduck_simulation"), "urdf", "soonduck.xacro"
)
assert xacro_file is not None, "Failed to parse Xacro file"

doc = xacro.process_file(xacro_file)
robot_desc = doc.toxml()

robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    output="screen",
    parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc}],
)
```
Gratefully, `robot_state_publisher` is pre-built in ROS2. You can use the code above to publish your file to `/robot_description` topic.

Plus, if you use `continuous` or `revolute` joint type, you have to use `joint_state_publisher`.

One thing you have to know is that `get_package_share_directory` function doesn't refer to your files inside your package. Instead, it refers to the directories from `install/package_name/share/package_name` folder in your workspace directory. Unfortunately, `colcon build` command might not copy every files to `install` folder unless it is specifically defined.
```
/soonduck_simulation
    /soonduck_simulation
    /launch
    /rviz
    /urdf
```
Your package directory might seems like this. `/launch`, `/rviz`, and `/urdf` folders, which are at the outside of the second `/soonduck_simulation` directory, is not going to be copied into `install` directory. In order to do that, you have to modify your `setup.sh` file in your package.
```
data_files=[
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
    (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
],
```
In `setup` function, find `data_files` argument and add last three lines. Those lines will include each folders to be built together. Don't forget to import necessary functions in `setup.sh` file.
```
import os
from glob import glob
```


# Dependencies

`gazebo`
```
$ sudo apt-get install ros-galactic-gazebo-ros
```

`xacro`
```
$ sudo apt-get install ros-galactic-xacro
```

`joint_state_publisher`
```
$ sudo apt-get install ros-galactic-joint-state-publisher
```

`teleop-twist-keyboard`

```
$ sudo apt-get install ros-galactic-teleop-twist-keyboard
```