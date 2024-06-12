# Signal Generator for my_rviz2_plugin

This program is designed for testing the `my_rviz2_plugin` package.

## Features

- **Random Signal Generation**: Generates a 12-bit random binary signal.
- **Signal Publishing**: Publishes the generated signal to the ROS 2 `/signals` topic at regular intervals.
- **Test Data Provision**: Serves as test data for verifying and debugging the `my_rviz2_plugin`.

## Usage

1. Set up your ROS 2 environment.
2. Run this script to start the signal generator node.

```
bash
ros2 run my_rviz2_plugin signal_generator.py
```


3. Subscribe to the `/signals` topic to observe the generated signals.

## Dependencies

- ROS 2
- `rclpy` package
- `std_msgs` package

## License

This project is licensed under the [Apache License 2.0](LICENSE).
