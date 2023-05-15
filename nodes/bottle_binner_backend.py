#!/usr/bin/env python3

import sys
from random import randint
from time import sleep
import numpy as np

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import ros2_numpy

from ros_trees_examples.action import ActuateGripper, MoveToNamedPose, MoveToPose
from ros_trees_examples.msg import Object
from ros_trees_examples.srv import FindObjects, GetSyncedImages


class DummyBottleBinnerBackend(Node):
    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480

    def __init__(self):
        super().__init__('dummy_bottle_binner_backend')

        # Startup all of our fake services...
        self._actuate_gripper_as = ActionServer(
            self, ActuateGripper, '/action/actuate_gripper', self.cb_actuate_gripper)
        self._detect_bottles_service = self.create_service(
            FindObjects, '/service/detect_bottles', self.cb_detect_bottles)
        self._synced_rgbd_service = self.create_service(
            GetSyncedImages, '/service/get_synced_rgbd', self.cb_get_synced_rgbd)
        self._move_gripper_location_as = ActionServer(
            self, MoveToNamedPose, '/action/move_gripper/location', self.cb_move_gripper_location)
        self._move_gripper_pose_as = ActionServer(
            self, MoveToPose, '/action/move_gripper/pose', self.cb_move_gripper_pose)

        print("All dummy services are now running...")

    @staticmethod
    def _print_dramatic(string, delay):
        print(string + " ... ", end="")
        sys.stdout.flush()
        sleep(delay)
        print("Done")

    def cb_actuate_gripper(self, goal):
        # Print some dummy output, & return an appropriate result
        self._print_dramatic(
            "%s gripper" % ("Opening" if goal.state
                            == ActuateGripper.Goal.STATE_OPEN else "Closing"),
            0.5)
        self._actuate_gripper_as.set_succeeded(ActuateGripper.Result())

    def cb_detect_bottles(self, request, response):
        # Generate some random detections within the images...
        depth_image = ros2_numpy.numpify(request.input_depth_image)
        rgb_image = ros2_numpy.numpify(request.input_rgb_image)
        objects = []
        for _ in range(0, randint(1, 5)):
            wh = (randint(5, 50), randint(5, 50))  # (w, h)
            xy = (randint(0, self.IMAGE_WIDTH - wh[0]),
                  randint(0, self.IMAGE_HEIGHT - wh[1]))  # (x, y)
            depth = ros2_numpy.msgify(Image,
                                     depth_image[xy[1]:xy[1] + wh[1],
                                                 xy[0]:xy[0] + wh[0]],
                                     encoding='mono8')
            depth.header.frame_id = request.input_depth_image.header.frame_id
            rgb = ros2_numpy.msgify(Image,
                                   rgb_image[xy[1]:xy[1] + wh[1],
                                             xy[0]:xy[0] + wh[0], :],
                                   encoding='rgb8')
            rgb.header.frame_id = request.input_rgb_image.header.frame_id
            objects.append(
                Object(class_label='bottle',
                       x_left=xy[0],
                       y_top=xy[1],
                       width=wh[0],
                       height=wh[1],
                       cropped_depth=depth,
                       cropped_rgb=rgb))
        self._print_dramatic("Detecting bottles", 0.1)
        response.objects = objects
        return response

    def cb_get_synced_rgbd(self, request, response):
        # Return a response with some dummy images
        self._print_dramatic("Getting synced RGBD", 0.1)
        depth_msg = ros2_numpy.msgify(
            Image,
            np.random.randint(100, 255,
                              (self.IMAGE_HEIGHT, self.IMAGE_WIDTH)).astype(
                                  np.uint8),
            encoding='mono8')
        depth_msg.header.frame_id = request.camera_namespace
        rgb_msg = ros2_numpy.msgify(
            Image,
            np.random.randint(0, 255,
                              (self.IMAGE_HEIGHT, self.IMAGE_WIDTH, 3)).astype(
                                  np.uint8),
            encoding='rgb8')
        rgb_msg.header.frame_id = request.camera_namespace
        response.synced_depth_image = depth_msg
        response.synced_rgb_image = rgb_msg
        return response

    def cb_move_gripper_location(self, goal):
        success = goal.pose_name in ['bin', 'workspace']
        if success:
            self._print_dramatic(
                "Moving gripper to location '%s'" % goal.pose_name, 5)
        else:
            print("I do not know the location '%s' ... Aborting" %
                  goal.pose_name)

        # Return an appropriate result
        (self._move_gripper_location_as.set_succeeded if success else
         self._move_gripper_location_as.set_aborted)(MoveToNamedPose.Result())

    def cb_move_gripper_pose(self, goal):
        pos = goal.goal_pose.pose.position
        self._print_dramatic(
            "Moving gripper to location (%f, %f, %f) in frame '%s'" %
            (pos.x, pos.y, pos.z, goal.goal_pose.header.frame_id), 5)

        # Return an appropriate result
        self._move_gripper_pose_as.set_succeeded(MoveToPose.Result())


def main(args=None):
    rclpy.init(args=args)
    dbbs = DummyBottleBinnerBackend()
    rclpy.spin(dbbs)
    dbbs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()