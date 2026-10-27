#!/usr/bin/env python3
"""Republica a atitude do FAST-LIO (/Odometry) como sensor_msgs/Imu no frame body, para o GSeg3D.

Com use_imu_orientation o GSeg nivela a nuvem pela orientation da IMU e procura na TF o frame da IMU ate o
robot_frame. Com robot_frame body e esta IMU tambem em body, a TF e a identidade e nada depende da arvore do
robo (cuja raiz, na simulacao, e o odom de ground truth). O camera_init do FAST-LIO e alinhado a gravidade
(patch em IMU_Processing.hpp), entao a orientation do /Odometry e a atitude do body em relacao a gravidade.
O carimbo e o da odometria, o mesmo da nuvem do quadro: o sincronizador do GSeg casa os dois sem folga.

  fastlio_attitude_imu.py [odometria] [saida]   (padrao /Odometry -> /terrain/body_attitude)
"""

import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def main():
    odom_topic = sys.argv[1] if len(sys.argv) > 1 else "/Odometry"
    out_topic = sys.argv[2] if len(sys.argv) > 2 else "/terrain/body_attitude"
    rclpy.init()
    node = rclpy.create_node("fastlio_attitude_imu")
    pub = node.create_publisher(Imu, out_topic, qos_profile_sensor_data)

    def on_odom(msg):
        imu = Imu()
        imu.header.stamp = msg.header.stamp
        imu.header.frame_id = msg.child_frame_id  # "body"
        imu.orientation = msg.pose.pose.orientation
        imu.angular_velocity_covariance[0] = -1.0     # sem giro nem aceleracao: so a atitude
        imu.linear_acceleration_covariance[0] = -1.0
        pub.publish(imu)

    node.create_subscription(Odometry, odom_topic, on_odom, 20)
    node.get_logger().info(f"{odom_topic} -> {out_topic} (orientation do body)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
