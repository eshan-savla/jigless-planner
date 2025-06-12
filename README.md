# jigless-planner

## Overview

`jigless-planner` is a ROS 2 package for hierarchical online planning and executing welding tasks in a jigless manufacturing scenario using the PlanSys2 framework.

## Quick Start
Docker images of this package are publicly available on Docker Hub and are the recommended way to run and test this package. You can pull the latest image with:

```sh
docker pull eshansavla0512/jigless-planner:run-1.0
```
### Running the framework

1. Start the container and launch the framework:
    ```sh
    docker run -it --rm --name jigless-planner eshansavla0512/jigless-planner:run-1.0
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch jigless-planner weldcell_example_launch.py
    ```
2. Open a new terminal and run the following command to start the action dummies:
    ```sh
    docker exec -it jigless-planner bash
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch jigless-planner dummies.launch.py
    ```
3. Open another terminal and run the following command to run an example:
    ```sh
    docker exec -it jigless-planner bash
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 run jigless_planner test1_client --ros-args -p joint_count:=10
    ```
The docker container can also be used similarly to run the tests from the [Testing](#testing) section.
## Building

1. Install all dependencies (from your workspace src folder):

    ```sh
    rosinstall ./jigless-planner/.rosinstall 
    ```

2. Build the package:

    ```sh
    colcon build --symlink-install --packages-select jigless-planner
    ```

3. Source your workspace after building:

    ```sh
    source install/setup.bash
    ```

## Running

Launch files are provided in the `launch/` directory to start the planner and related nodes.

### Example: Weldcell Scenario

To launch the full weldcell example (including planners and controllers):

```sh
ros2 launch jigless-planner weldcell_example_launch.py
```

### Other Launch Files

- `controllers.launch.py`: Launches only the top controller node.
- `dummies.launch.py`: Launches dummy action servers for testing.
- `test1_complexity.launch.py`, `test1_adaptability.launch.py`, `test2_robustness.launch.py`: Launch various test scenarios for benchmarking and robustness.

To check the arguments of any launch file, you can use:

```sh
ros2 launch jigless-planner <launch_file_name>.py --show-args
```

To use any launch file:

```sh
ros2 launch jigless-planner <launch_file_name>.py
```
## Testing
To run the tests for this package run the following commands in different terminals:
### Test 1: Complexity
```sh
ros2 launch jigless-planner test1_complexity.launch.py
ros2 launch jigless-planner dummies.launch.py
ros2 run jigless_planner test1_client --ros-args -p joint_count:=<no_of_joints>
```
### Test 2: Adaptability
- Set the action desired implementation under `src/dummies/`directory to fail at joint6 and rebuild the package.
- next run:
```sh
ros2 launch jigless-planner test1_adaptability.launch.py
ros2 launch jigless-planner dummies.launch.py
ros2 run jigless_planner test1_client --ros-args -p joint_count:=10
```
### Test 2: Robustness
- Unset any forced failures in the `src/dummies/` directory.
- next run:
```sh
ros2 launch jigless-planner test2_robustness.launch.py
ros2 launch jigless-planner dummies.launch.py
ros2 run jigless_planner test2_client_node --ros-args -p frequency:=<frequency> -p delay:=<delay> -p duration:=<duration>
```

## Configuration

- **PDDL files**: Located in the `pddl/` directory.
- **Behavior trees**: Located in `behavior_trees_xml/`.
- **Parameters**: Located in `config/`.

Scripts have been provided in the `scripts/` directory to help with generating PDDL files and behavior tree configs. Also contains a script to count topological sorts of a given pddl problem file.