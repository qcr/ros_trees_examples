#!/usr/bin/env python3

import numpy as np
from random import randint
from py_trees.composites import Sequence
from py_trees.decorators import FailureIsSuccess, SuccessIsRunning

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import ros2_numpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Vector3

import ros_trees.data_management as dm
from ros_trees.leaves import Leaf
from ros_trees.leaves_ros import ActionLeaf, ServiceLeaf
from ros_trees.trees import BehaviourTree

from ros_trees_examples.action import ActuateGripper, MoveToNamedPose, MoveToPose
from ros_trees_examples.srv import FindObjects, GetSyncedImages

################################################################################
############################### Leaf definitions ###############################
################################################################################


class _ActuateGripper(ActionLeaf):

    def __init__(self, *args, **kwargs):
        super(_ActuateGripper, self).__init__(
            action_namespace='/action/actuate_gripper',
            action_class=ActuateGripper,
            *args,
            **kwargs)


class CloseGripper(_ActuateGripper):
    CLOSE_GOAL = ActuateGripper.Goal(state=ActuateGripper.Goal.STATE_CLOSED)

    def __init__(self, *args, **kwargs):
        super(CloseGripper, self).__init__(
            name="Close Gripper",
            load_value=CloseGripper.CLOSE_GOAL,
            *args,
            **kwargs)


class GetSyncedRgbd(ServiceLeaf):

    def __init__(self, *args, **kwargs):
        super(GetSyncedRgbd, self).__init__(
            name="Get Synced RGBD",
            service_name='/service/get_synced_rgbd',
            service_class=GetSyncedImages,
            save=True,
            *args,
            **kwargs)


class DetectBottles(ServiceLeaf):

    def __init__(self, *args, **kwargs):
        super(DetectBottles,self).__init__(
            name="Detect bottles",
            service_name='/service/detect_bottles',
            service_class=FindObjects,
            save=True,
            *args,
            **kwargs)


class MoveGripperToPose(ActionLeaf):

    def __init__(self, *args, **kwargs):
        super(MoveGripperToPose, self).__init__(
            name="Move gripper to pose",
            action_namespace='/action/move_gripper/pose',
            action_class=MoveToPose,
            *args,
            **kwargs)


class MoveGripperToLocation(ActionLeaf):

    def __init__(self, *args, **kwargs):
        super(MoveGripperToLocation, self).__init__(
            name="Move gripper to location",
            action_namespace='/action/move_gripper/location',
            action_class=MoveToNamedPose,
            *args,
            **kwargs)


class OpenGripper(_ActuateGripper):
    OPEN_GOAL = ActuateGripper.Goal(state=ActuateGripper.Goal.STATE_OPEN)

    def __init__(self, *args, **kwargs):
        super(OpenGripper, self).__init__(
            name="Open Gripper",
            load_value=OpenGripper.OPEN_GOAL,
            *args,
            **kwargs)


class PopFromList(Leaf):

    def __init__(self, pop_position=0, *args, **kwargs):
        super(PopFromList, self).__init__(
            name="Pop from list",
            result_fn=self._pop_item,
            *args,
            **kwargs)
        self.pop_position = pop_position

    def _pop_item(self):
        if not self.loaded_data:
            return None
        item = self.loaded_data.pop(self.pop_position)
        if self.load_key is not None:
            dm.set_value(self.load_key, self.loaded_data)
        else:
            dm.set_last_value(self, self.loaded_data)
        return item


class Print(Leaf):

    def __init__(self, *args, **kwargs):
        super(Print, self).__init__(
            name="Print",
            result_fn=self._print,
            *args,
            **kwargs)

    def _print(self):
        print(self.loaded_data)
        return True


class PrintObjects(Leaf):

    def __init__(self, *args, **kwargs):
        super(PrintObjects, self).__init__(
            name="Print Objects",
            result_fn=self._print_objects,
            *args,
            **kwargs)

    def _print_objects(self):
        if self.loaded_data is None or not self.loaded_data:
            print("The detector found no objects!")
        else:
            print(
                "The detector found %d objects at the following coordinates:" %
                len(self.loaded_data))
            for o in self.loaded_data:
                print(
                    "\t'%s' of pixel dimensions %dx%d @ top left coordinates:"
                    " (%d,%d)" %
                    (o.class_label, o.width, o.height, o.x_left, o.y_top))

        return True


class WaitForEnterKey(Leaf):

    def __init__(self, *args, **kwargs):
        super(WaitForEnterKey, self).__init__(
            name="Wait for Enter Key",
            result_fn=self._wait_for_enter,
            *args,
            **kwargs)

    def _wait_for_enter(self):
        # NOTE: this is blocking within a leaf ... typically BAD
        input(self.loaded_data if self.
              loaded_data else "Press enter to continue: ")
        return True


################################################################################
############################## Branch definitions ##############################
################################################################################


class BinItem(Sequence):

    def __init__(self, load_pose_key, load_pose_fn=None, *args, **kwargs):
        super(BinItem, self).__init__("Bin Item", [
            OpenGripper(),
            MoveGripperToLocation(load_value='workspace'),
            MoveGripperToPose(load_key=load_pose_key, load_fn=load_pose_fn),
            CloseGripper(),
            MoveGripperToLocation(load_value='bin')
        ])


################################################################################
######################### Tree definition & Execution ##########################
################################################################################


def object_list_from_response(leaf, response):
    return leaf._default_save_fn(response.objects)


def pose_from_object(leaf):
    # Hacky, & NOT a good way to convert image coordinates to pose... but will
    # do for the sake of our example
    _FOVS = np.deg2rad(np.array([85, 50]))  # Horizontal & vertical FOV
    _IMAGE_DIMS = np.array([640.0, 480.0])  # (w, h)
    object_msg = super(ActionLeaf, leaf)._default_load_fn()

    # Derive pose from depth of detection midpoint, & dumb FOV angle division
    cropped_depth = ros2_numpy.numpify(object_msg.cropped_depth)
    r = 0.01 * cropped_depth[cropped_depth.shape[0] // 2,
                             cropped_depth.shape[1] // 2]
    angles = _FOVS * (
        (np.array([object_msg.x_left, object_msg.y_top]) + 0.5 *
         np.array([object_msg.width, object_msg.height])) / _IMAGE_DIMS - 0.5)
    xyz = r * np.array([
        np.cos(angles[0]) * np.sin(np.pi / 2 - angles[1]),
        np.sin(angles[0]) * np.sin(np.pi / 2 - angles[1]),
        np.cos(np.pi / 2 - angles[1])
    ])
    xyz_uv = xyz / np.linalg.norm(xyz)
    q = np.concatenate((np.sin(0) * np.cos(xyz_uv), np.array([np.cos(0)])))

    # Autogenerate a goal message for the ActionLeaf from the Pose
    return dm.auto_generate(PoseStamped(
        header=Header(frame_id='camera_wrist'),
        pose=Pose(position=Vector3(*xyz.tolist()),
                  orientation=Quaternion(*q.tolist()))),
                            type(leaf._action_class().action_goal.goal),
                            breakdown=False)



if __name__ == '__main__':
    rclpy.init()

    # Behaviour tree is itself a Node... Ensure it is spun after setup.
    tree = BehaviourTree("Bottle Binner",
        Sequence("Bin Bottles", children=[
            GetSyncedRgbd(load_value='camera_wrist'),
            DetectBottles(save_key='bottles', save_fn=object_list_from_response),
            PrintObjects(load_key='bottles'),
            FailureIsSuccess(
                name="F=S",
                child=SuccessIsRunning(
                    name="S=R",
                    child=Sequence("Bin bottle", children=[
                            PopFromList(load_key='bottles', save_key='bottle'),
                            Print(load_value="Binning bottle..."),
                            BinItem(
                                load_pose_fn=pose_from_object,
                                load_pose_key='bottle'),
                            Print(load_value="Successfully binned bottle!")
                        ]
                    )
                )
            ),
            WaitForEnterKey(load_value="All bottles binned! Press enter to restart ... ")]
        )
    )

    if not tree.run():
        rclpy.shutdown()
        exit(1)
    
    rclpy.spin(tree.node)

    print("Complete... Exiting!")
    rclpy.shutdown()
    exit(0)