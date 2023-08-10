# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import carla
import time
import math
from scipy.spatial.transform import Rotation 
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped, AccelWithCovarianceStamped, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from tier4_system_msgs.msg import ModeChangeAvailable

class GTLocalizer(Node):
    # CARLA_HOST = "172.17.0.1"
    CARLA_HOST = "10.176.36.40"
    SOURCE_FRAME = "map"
    TARGET_FRAME = "base_link"
    ROLE_NAME = "hero"
    INITIAL_POSE_TOPIC = "/initialpose"
    POSE_TOPIC = "/localization/pose_twist_fusion_filter/pose"
    TWIST_TOPIC = "/localization/pose_twist_fusion_filter/twist"
    ACC_TOPIC = "/localization/acceleration"
    INITIAL_STATE_TOPIC = "/api/localization/initialization_state"
    KINEMATIC_STATE_TOPIC = "/localization/kinematic_state"
    LOCALIZATION_READY_TOPIC = "/system/component_state_monitor/component/autonomous/localization"

    def __init__(self):
        super().__init__('ground_truth_localizer')
        
        self.intialpose_subscriber_ = self.create_subscription(PoseWithCovarianceStamped, GTLocalizer.INITIAL_POSE_TOPIC, self.on_initalpose, 10)

        self.pose_publisher_ = self.create_publisher(PoseStamped, GTLocalizer.POSE_TOPIC, 10)
        self.twist_publisher_ = self.create_publisher(TwistStamped, GTLocalizer.TWIST_TOPIC, 10)
        self.acc_publisher_ = self.create_publisher(AccelWithCovarianceStamped, GTLocalizer.ACC_TOPIC, 10)

        ready_publisher_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth = 3
        )
        self.ready_publisher_ = self.create_publisher(ModeChangeAvailable, GTLocalizer.LOCALIZATION_READY_TOPIC, ready_publisher_profile)

        initial_state_publisher_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth = 3
        )
        self.initial_state_publisher_ = self.create_publisher(LocalizationInitializationState, GTLocalizer.INITIAL_STATE_TOPIC, initial_state_publisher_profile)

        kinematic_state_publisher_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability = QoSDurabilityPolicy.VOLATILE,
            depth = 3
        )
        self.kinematic_state_publisher_ = self.create_publisher(Odometry, GTLocalizer.KINEMATIC_STATE_TOPIC, kinematic_state_publisher_profile)

        self.tf_broadcaster_ = TransformBroadcaster(self)
        self.ego_vehicle = None 
        self.carla_client = carla.Client(GTLocalizer.CARLA_HOST, 2000)
        self.world = self.carla_client.get_world()
        self.initialized = False

        """Get the ego vehicle from the world."""
        ego_found = False
        while True:
            actors = self.world.get_actors().filter('*vehicle*')
            for car in actors:
                if car.attributes['role_name'] == GTLocalizer.ROLE_NAME:
                    self.ego_vehicle = car
                    ego_found = True
                    break
            if ego_found:
                self.get_logger().info("Ego found.")
                break
            else:
                self.get_logger().warn('Waiting for ego vehicle...')
                time.sleep(1)        
                
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
            
    def on_initalpose(self, msg):
        pose = msg.pose.pose
        transform = carla.Transform(carla.Location(pose.position.x, -pose.position.y, pose.position.z), carla.Rotation(0, 0, 0))
        transform.rotation.yaw = -Rotation.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_euler('xyz', degrees=True)[2]
        self.get_logger().info("Setting initial pose to %s"%transform)
        self.ego_vehicle.set_transform(transform)
        initialization_state = LocalizationInitializationState()
        stamp = self.get_clock().now().to_msg()
        initialization_state.stamp = stamp 
        initialization_state.state = 3
        self.initialized = True
        self.initial_state_publisher_.publish(initialization_state)


    def get_pose_from_carla(self):
        transform = self.ego_vehicle.get_transform()
        posestamped = PoseStamped()
        posestamped.header.frame_id = GTLocalizer.SOURCE_FRAME
        posestamped.pose.position.x = transform.location.x
        posestamped.pose.position.y = -transform.location.y
        posestamped.pose.position.z = transform.location.z
        orientation = Rotation.from_euler('xyz', [transform.rotation.roll, transform.rotation.pitch, -transform.rotation.yaw], degrees=True).as_quat()
        posestamped.pose.orientation.x = orientation[0]
        posestamped.pose.orientation.y = orientation[1]
        posestamped.pose.orientation.z = orientation[2]
        posestamped.pose.orientation.w = orientation[3]
        # self.get_logger().info('Publishing: "%s"' % posestamped)
        return posestamped

    def get_twist_from_carla(self):
        linear_velocity = self.ego_vehicle.get_velocity()
        angular_velocity = self.ego_vehicle.get_angular_velocity()
        twiststamped = TwistStamped()
        twiststamped.twist.linear.x = math.sqrt(linear_velocity.x**2 + linear_velocity.y**2 + linear_velocity.z**2)
        twiststamped.twist.angular.x = angular_velocity.x
        twiststamped.twist.angular.y = angular_velocity.y
        twiststamped.twist.angular.z = angular_velocity.z
        # self.get_logger().info('Publishing: "%s"'%linear_velocity)
        return twiststamped

    def get_acc_from_carla(self):
        acceleration = self.ego_vehicle.get_acceleration()
        accstamped = AccelWithCovarianceStamped()
        accstamped.accel.accel.linear.x = math.sqrt(acceleration.x**2 + acceleration.y**2 + acceleration.z**2)
        # self.get_logger().info('Publishing: "%s"'%acceleration)
        return accstamped

    def publish_tf(self, pose):
        transform = TransformStamped()
        transform.header.stamp = pose.header.stamp
        transform.header.frame_id = GTLocalizer.SOURCE_FRAME
        transform.child_frame_id = GTLocalizer.TARGET_FRAME
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation.x = pose.pose.orientation.x
        transform.transform.rotation.y = pose.pose.orientation.y
        transform.transform.rotation.z = pose.pose.orientation.z
        transform.transform.rotation.w = pose.pose.orientation.w
        self.tf_broadcaster_.sendTransform(transform)

    def timer_callback(self):
        pose = self.get_pose_from_carla()
        twist = self.get_twist_from_carla()
        acc = self.get_acc_from_carla()
        stamp = self.get_clock().now().to_msg()
        # self.get_logger().info("Time is : %s"%stamp)
        pose.header.stamp = stamp
        twist.header.stamp = stamp
        acc.header.stamp = stamp

        self.publish_tf(pose)
        
        if not self.initialized:
            initialization_state = LocalizationInitializationState()
            initialization_state.stamp = stamp
            initialization_state.state = 3
            self.initialized = True
            self.initial_state_publisher_.publish(initialization_state)
        self.pose_publisher_.publish(pose)
        self.twist_publisher_.publish(twist)
        self.acc_publisher_.publish(acc)
        
        
        ready = ModeChangeAvailable()
        ready.stamp = stamp
        ready.available = True
        self.ready_publisher_.publish(ready)
        
        kinematic_state = Odometry()
        kinematic_state.header.stamp = stamp
        kinematic_state.header.frame_id = GTLocalizer.SOURCE_FRAME
        kinematic_state.child_frame_id = GTLocalizer.TARGET_FRAME
        kinematic_state.pose.pose = pose.pose
        kinematic_state.twist.twist = twist.twist
        self.kinematic_state_publisher_.publish(kinematic_state)

def main(args=None):
    rclpy.init(args=args)
    gt_localizer = GTLocalizer()
    rclpy.spin(gt_localizer)
    gt_localizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
