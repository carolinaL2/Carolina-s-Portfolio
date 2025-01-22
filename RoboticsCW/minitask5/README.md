# Quick commands for demo

1. Symbo-link into catkin
    ```
    ln -s ~/PATH/g21/minitask5 ~/catkin_ws/src/minitask5
    ```


2. Source devel and make
    ```
    source ~/catkin_ws/devel/setup.bash
    cd  ~/catkin_ws && catkin_make
    ```


3. Launch gazebo
    ```
    roslaunch minitask5 turtlebot3_training.launch
    ```
4. Launch RVIZ Nav
    ```
    roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=`rospack find minitask5`/maps/train_env.yaml
    ```

5. **REMEMBER TO ENABLE MARKERS**