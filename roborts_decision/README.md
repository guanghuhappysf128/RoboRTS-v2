For guys who are looking for allowing thir agent to detect enemy robots by its own camera in simulation world, here are the instructions for making use of `armor_detection` package in gazebo simulation environment:
0. First of all, do fetch the latest commits in most active branches in `gazebo_temp`(master) and `RoboRTS-v2`(devel) repos and merge into your local repo. Follow the newest instruction in the ReadMe files of `gazebo_temp`.
1. Modify relevant config files: 
    * `constraint_set_r1.prototxt(optional)`:  
        PATH: /RoboRTS-v2/roborts_detection/armor_detection/constraint_set/config/   
        Detail: If you want to check the actual image received from the camera, switch the value of `enable_debug` to `true`
    * `infantry.gazebo.xacro`:  
        PATH: /gazebo_temp/RoboRTS_Gazebo/robots/  
        Detail: At `line 192`, under the block named `camera`, the `sensor` element, change the value of `name` attribute from `camera1` to `mercure_camera`.
2. Establish armor detection module: Run `armor_detection_node` and `armor_detection_client`:
    * Establish simulation environment:
        ```bash
        roslaunch roborts_bringup multibots_sim.launch
        ```
    * Run `armor_detection_node` on robot `r1`:
        ```bash
        ROS_NAMESPACE='/r1' rosrun roborts_detection armor_detection_node
        ```
        Then you should see something like information like "Constrain set  done!"
    * Run `armor_detection_client` on robot `r1`:
        ```bash
        ROS_NAMESPACE='/r1' rosrun roborts_detection armor_detection_client
        ```
        Now you should see a prompt saying "Please send a command" and four options, then input 1
    * Now if you switched the value of `enable_debug` to `true` above, you should see window displaying a video stream, that's exactly the vision captured by robot 'r1' in gazebo simulation 
3. Check the information sent from armor_detection_node
    * Check relative topics and services
        ```bash
        rosnode info /r1/armor_detection_node
        ```
        Then you should be able to see all its publications, subscriptions and services, we mainly focus on the topic it publishes named `/r1/cmd_gimbal_angle`
    * Check the protocol of `cmd_gimbal_angle` topic
        ```bash
        rostopic type /r1/cmd_gimbal_angle  
        // "roborts_msgs/GimbalAngle"
        rosmsg show roborts_msgs/GimbalAngle 
        //  bool yaw_mode
            bool pitch_mode
            float64 yaw_angle
            float64 pitch_angle
        ```
        As indicated above, the subscriber of the topic `/r1/cmd_gimbal_angle` should receive four values from it and we mainly need the latter two values, which stands for the relative angles between the detected enemy armor board and the gimbal plane. Currently we don't really care about the specific value of it. If one of those values is `non-zero`, then it means "enemy detected".
4. Set up a scenario for testing whether armor_detection node is working:
    * You can change the description `.launch` files to set up a 1v1 scenario and test the performance of the `armor_detection_node`. In gazebo GUI, there is a tool in toolbar named `"translation mode"`, you can use it to drag a robot to anywhere you want.