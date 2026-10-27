#!/usr/bin/env python3
"""Espera o robo ficar parado, pela IMU, antes de subir o FAST-LIO.

O FAST-LIO estima a gravidade com ~10 amostras de IMU (~0,1 s) no primeiro quadro. Se o robo ainda
estiver se acomodando (na simulacao: a queda do spawn), o camera_init nasce inclinado; num teste na
encosta de 13,6 graus o pitch saiu 4,5 graus errado. Parado = numa janela de WINDOW s (tempo das
mensagens), |giro| < GYRO_MAX e desvio padrao de |aceleracao| < ACC_STD_MAX.

  wait_imu_still.py [topico_imu]     (padrao /livox/imu; sai com 1 se estourar TIMEOUT s de relogio)
"""

import collections
import math
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

WINDOW = 2.0         # [s]
GYRO_MAX = 0.02      # [rad/s]  ruido da IMU simulada: 1e-3 por eixo
ACC_STD_MAX = 0.05   # [m/s^2]  ruido da IMU simulada: 2e-2 por eixo
TIMEOUT = 180.0      # [s]


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/livox/imu"
    rclpy.init()
    node = rclpy.create_node("wait_imu_still")
    samples = collections.deque()
    state = {"still": False}

    def on_imu(msg):
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        w, a = msg.angular_velocity, msg.linear_acceleration
        samples.append((t, math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z), math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)))
        while samples and t - samples[0][0] > WINDOW:
            samples.popleft()
        if t - samples[0][0] < 0.95 * WINDOW:
            return
        acc = [s[2] for s in samples]
        mean = sum(acc) / len(acc)
        acc_std = math.sqrt(sum((x - mean) ** 2 for x in acc) / len(acc))
        state["still"] = max(s[1] for s in samples) < GYRO_MAX and acc_std < ACC_STD_MAX

    node.create_subscription(Imu, topic, on_imu, qos_profile_sensor_data)
    print(f"wait_imu_still: esperando o robo parar ({topic})...", flush=True)
    start = time.time()
    while rclpy.ok() and not state["still"] and time.time() - start < TIMEOUT:
        rclpy.spin_once(node, timeout_sec=0.1)
    ok = state["still"]
    print(f"wait_imu_still: {'parado' if ok else 'TIMEOUT, seguindo mesmo assim'} apos {time.time() - start:.1f} s",
          flush=True)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
