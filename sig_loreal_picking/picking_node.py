#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from object_detector_tensorflow.client import Client as DetectorClient
from robot_interface_eki.client import Client as RobotClient
from point_transformation.client import Client as TransformClient
from sensor_msgs.msg import Image, RegionOfInterest


class PickingNode(Node):

    def __init__(self,
                 name: str = "picking_node",
                 image_topic: str = "stereo/left/image_rect_color",
                 depth_image_topic: str = "stereo/depth",
                 result_topic: str = "picking_node/result_image"):

        super().__init__(name)

        self.image = None
        self.depth_image = None
        self._current_image = None
        self._current_depth_image = None

        self.detector_client = DetectorClient(self)
        self.transform_client = TransformClient(self)
        self.robot_client = RobotClient(self, base_index=4, tool_index=4)

        self.result_image_publisher = self.create_publisher(Image,
                                                            result_topic,
                                                            10)

        self._image_subscription = self.create_subscription(Image,
                                                            image_topic,
                                                            self._image_callback,
                                                            10)

        self._depth_image_subscription = self.create_subscription(Image,
                                                                  depth_image_topic,
                                                                  self._depth_image_callback,
                                                                  10)

    def _image_callback(self, msg):

        self._current_image = msg

    def _depth_image_callback(self, msg):

        self._current_depth_image = msg

    def _images_recieved(self):
        if self._current_image is not None and self._current_depth_image is not None:
            self.image = self._current_image
            self.depth_image = self._current_depth_image

            print("images recieved")

            return True

        return False

    def wait_for_new_images(self):
        self._current_image = None
        self._current_depth_image = None

        while rclpy.ok() and not self._images_recieved():
            rclpy.spin_once(self)


def main(args=None):

    rclpy.init(args=args)

    node = PickingNode(name='picking_node')

    node.wait_for_new_images()

    ################# Get Detections #########################

    # (Optional) Only search objects in this region of interest
    # roi = RegionOfInterest(x_offset=0,
    #                        y_offset=0,
    #                        height=1920,
    #                        width=1080)

    roi = RegionOfInterest()

    detections, result_image = node.detector_client.detect_objects(
        node.image, roi)

    node.result_image_publisher.publish(result_image)

    ################## Get Coordinates ########################

    object_pixels = []

    for detection in detections:
        print(
            detection.class_id,
            detection.class_name,
            detection.probability,
            detection.bounding_box)

        object_pixels.append(
            (detection.bounding_box.x_offset +
             int(detection.bounding_box.width / 2),
             detection.bounding_box.y_offset +
             int(detection.bounding_box.height / 2)))

    object_positions = []

    for pixel in object_pixels:

        object_positions.append(
            node.transform_client.pixel_to_point(pixel, node.depth_image))

    print(object_positions)

    ################# Move Robot to Coordinates #######################

    for position in object_positions:
        node.robot_client.move(
            cartesian=[position.x, position.y, position.z, 90.0, 0.0, 0.0], lin=True)

    # node.robot_client.move(
    #     cartesian=[0.0, 600.0, 800.0, 90.0, 0.0, 0.0], lin=False)
    # node.robot_client.move(
    #     cartesian=[0.0, 400.0, 800.0, 90.0, 0.0, 0.0], lin=True)
    # node.robot_client.move(
    #     cartesian=[0.0, 600.0, 800.0, 90.0, 0.0, 0.0], lin=True)

    # node.robot_client.move(cartesian=[114.0, 535.0, 365.0, 0.0, 0.0, 0.0], lin=False)
    # node.robot_client.move(cartesian=[93.0, 165.0, 365.0, 0.0, 0.0, 0.0], lin=True)
    # node.robot_client.move(target=5)
    # node.robot_client.move(joints=[0.2, 1.0, 2.2, 3.4, 1.0, 1.0, 1.0])
    # node.robot_client.move(cartesian=[0.2, 1.0, 2.2, 3.4, 1.0, 1.0], lin=False)

    # node.robot_client.grip(item_size=8.3, close=True)
    # node.robot_client.grip(suction_active=True, cylinder_position=1.8)
    # node.robot_client.grip(item_size=1.2, close=False, suction_active=False, cylinder_position=3.6)

    node.robot_client.run()

    ############################################################

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
