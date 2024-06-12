# my_rviz2_plugin

`my_rviz2_plugin` is a custom plugin for ROS2 RViz2. This plugin provides a panel to display signals.

## Requirements

- ROS2 Humble Hawksbill or later
- Qt5
- CMake 3.5 or later

## Installation

### Install Dependencies

Run the following commands to install the necessary dependencies:


bash
sudo apt update
sudo apt install -y \
ros-humble-rviz-common \
ros-humble-rclcpp \
ros-humble-std-msgs \
qt5-default



### Build the Package

Create a workspace, clone the package, and build it:

bash
mkdir -p ~/rviz2_plugin_ws/src
cd ~/rviz2_plugin_ws/src
git clone https://github.com/yourusername/my_rviz2_plugin.git
cd ~/rviz2_plugin_ws
source /opt/ros/humble/setup.bash
colcon build



## Usage

After the build is complete, you can launch RViz2 and use the plugin with the following commands:

bash
source ~/rviz2_plugin_ws/install/setup.bash
rviz2



Once RViz2 is running, go to the "Panels" menu on the left, select "Add New Panel", and add `my_rviz2_plugin/SignalPanel`.

## File Structure

- `src/`: Source code
- `include/`: Header files
- `CMakeLists.txt`: CMake build configuration
- `plugin_description.xml`: Plugin description file

## Contributing

Bug reports and pull requests are welcome. Please create an issue or send a pull request on the GitHub repository.

## License

This project is licensed under the Apache License 2.0. See the `LICENSE` file for details.
