#!/usr/bin/env python3

from logging import PlaceHolder

from matplotlib.transforms import Transform
import rclpy
from geometry_msgs.msg import Transform
import tf2_ros
from tf2_ros import TransformStamped
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates
from tf_transformations import *
import os
from functools import partial
import pysdf
from rclpy.time import Time


tfBroadcaster = None
submodelsToBeIgnored = []
lastUpdateTime = None
updatePeriod = 0.05
model_cache = {}


def is_ignored(link_name):
    for ignored_submodel in submodelsToBeIgnored:
        if link_name.startswith(ignored_submodel + '::'):
            return True
    return False


def on_link_states_msg(node, logger, link_states_msg):
    """
    Publish tf for each model in current Gazebo world
    """
    global lastUpdateTime
    sinceLastUpdateDuration = node.get_clock().now().to_msg().sec - lastUpdateTime
    if sinceLastUpdateDuration < updatePeriod:
        return
    lastUpdateTime = node.get_clock().now().to_msg().sec

    poses = {'gazebo_world': identity_matrix()}
    for (link_idx, link_name) in enumerate(link_states_msg.name):
        poses[link_name] = pysdf.pose_msg2homogeneous(link_states_msg.pose[link_idx])

    for (link_idx, link_name) in enumerate(link_states_msg.name):
        modelinstance_name = link_name.split('::')[0]
        model_name = pysdf.name2modelname(modelinstance_name)
        if not model_name in model_cache:
            sdf = pysdf.SDF(model=model_name)
            model_cache[model_name] = sdf.world.models[0] if len(sdf.world.models) >= 1 else None
            if model_cache[model_name]:
                logger.info('Loaded model: %s' % model_cache[model_name].name)
            else:
                logger.info('Unable to load model: %s' % model_name)
        model = model_cache[model_name]
        link_name_in_model = link_name.replace(modelinstance_name + '::', '')
        if model:
            link = model.get_link(link_name_in_model)
            if link.tree_parent_joint:
                parent_link = link.tree_parent_joint.tree_parent_link
                parent_link_name = parent_link.get_full_name()
                parentinstance_link_name = parent_link_name.replace(model_name, modelinstance_name, 1)
            else:  # direct child of world
                parentinstance_link_name = 'gazebo_world'
        else:  # Not an SDF model
            parentinstance_link_name = 'gazebo_world'
        if is_ignored(parentinstance_link_name):
            logger.info("Ignoring TF %s -> %s" % (parentinstance_link_name, link_name))
            continue
        pose = poses[link_name]
        parent_pose = poses[parentinstance_link_name]
        rel_tf = concatenate_matrices(inverse_matrix(parent_pose), pose)
        pose = pysdf.homogeneous2pose_msg(rel_tf)
        tf_message = TransformStamped()
        tf_message.header.stamp = node.get_clock().now().to_msg()
        tf_message.transform.translation.x = pose.position.x
        tf_message.transform.translation.y = pose.position.y
        tf_message.transform.translation.z = pose.position.z
        tf_message.transform.rotation.x = pose.orientation.x
        tf_message.transform.rotation.y = pose.orientation.y
        tf_message.transform.rotation.z = pose.orientation.z
        tf_message.transform.rotation.w = pose.orientation.w
        tf_message.header.frame_id = pysdf.sdf2tfname(parentinstance_link_name)
        tf_message.child_frame_id = pysdf.sdf2tfname(link_name)
        
        tfBroadcaster.sendTransform(tf_message)


def set_logger_format():
    os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "[{time}][{name}][{file_name}][{function_name}][{line_number}][{severity}]\n{message}\n"
    os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def main():
    set_logger_format()
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node(node_name="modelstate_2_jointstate")
    logger = node.get_logger()
    logger.set_level(level=10)

    global submodelsToBeIgnored
    submodelsToBeIgnored = node.get_parameter_or('~ignore_submodels_of', '').split(';')
    logger.info('Ignoring submodels of: ' + str(submodelsToBeIgnored))

    global tfBroadcaster
    tfBroadcaster = tf2_ros.TransformBroadcaster(node=node)

    global lastUpdateTime
    lastUpdateTime = node.get_clock().now().to_msg().sec
    on_link_states_msg_bound = partial(on_link_states_msg, node, logger)
    linkStatesSub = node.create_subscription(
        topic='link_states', msg_type=LinkStates, callback=on_link_states_msg_bound, qos_profile=10)

    logger.info('Spinning')
    rclpy.spin(node=node)


if __name__ == '__main__':
    main()
