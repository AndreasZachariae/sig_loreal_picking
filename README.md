# sig_loreal_picking

## Neccessary packages:

    git clone -b dev --recurse-submodules https://www.w.hs-karlsruhe.de/gitlab/iris/core/ros_core.git

    git clone -b dev --recurse-submodules https://www.w.hs-karlsruhe.de/gitlab/iris/common/robot_interface_eki.git

    git clone -b dev https://www.w.hs-karlsruhe.de/gitlab/iris/common/object_detector_tensorflow.git

    git clone -b dev https://www.w.hs-karlsruhe.de/gitlab/iris/common/point_transformation.git

    git clone https://github.com/AndreasZachariae/sig_loreal_picking.git


## How to run:

1. Terminal in ROS 1 with roscore

        source /opt/ros/melodic/setup.bash
        roscore

2. Terminal in ROS 1 for Roboception driver

        source /opt/ros/melodic/setup.bash
        rosrun rc_visard_driver rc_visard_driver

3. Terminal in ROS 2 for bridge between ROS 1 -> 2

        source /opt/ros/melodic/setup.bash
        source /opt/ros/dashing/setup.bash
        ros2 run ros1_bridge dynamic_bridge

4. Terminal in ROS 2 to launch all necessary services (robot_interface_eki, object_detector_tensorflow, point_transformation)

        source /opt/ros/dashing/setup.bash
        source ~/ros_ws/install/setup.bash
        ros2 launch sig_loreal_picking services.launch.py

5. Terminal in ROS 2 to start the detection and picking process

        source /opt/ros/dashing/setup.bash
        source ~/ros_ws/install/setup.bash
        ros2 run sig_loreal_picking picking_node

    Only restart this last picking_node for a new pick with the latest images

### (Optional) for only viewing the object detection output with bounding boxes:

Terminal 1 + 2 + 3 as above

4. Terminal

        source /opt/ros/dashing/setup.bash
        source ~/ros_ws/install/setup.bash
        ros2 launch object_detection_tensorflow continuous_detection.launch.py

5. Terminal 

        source /opt/ros/dashing/setup.bash
        rqt

    In rqt (if not already) select Plugins -> Vizualization -> Image View

    Select the image topic `continuous_detection_node/result_image` from the dropdown menu.
