#First time install

1. Clone the repo into your ros_ws/src folder
2. navigate to your ros_ws folder and run the followin command

`rosdep install --from-paths src --ignore-src -r -y`

#Building and running

1. Run the following commands

`colcon build --packages-select block_tam`
`. install/setup.bash`
`ros2 run block_tam block_tam`