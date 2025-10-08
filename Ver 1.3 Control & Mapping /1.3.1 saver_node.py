#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import subprocess
from std_msgs.msg import String
import os

def save_map_callback(message):
    """Fungsi ini dieksekusi setiap kali ada pesan masuk untuk menyimpan peta."""
    map_name = message.data
    rospy.loginfo(f"[SaverNode] Menerima perintah untuk menyimpan peta: {map_name}")

    if not map_name:
        rospy.logerr("[SaverNode] Nama peta kosong, perintah dibatalkan.")
        return

    # Perintah untuk membuka terminal baru dan menjalankan map_saver dengan lingkungan lengkap
    # Pastikan path ke catkin_ws dan nama paket sudah benar
    command_to_run = f"""
    gnome-terminal -- /bin/bash -c "source /opt/ros/noetic/setup.bash; \\
    source ~/catkin_ws/devel/setup.bash; \\
    echo '[INFO] Sedang menyimpan peta dengan nama: {map_name}'; \\
    rosrun map_server map_saver -f ~/catkin_ws/src/autonomus_mobile_robot/maps/{map_name}; \\
    echo '[INFO] Perintah Selesai. Terminal akan ditutup dalam 5 detik...'; \\
    sleep 5; exit"
    """
    
    try:
        # Jalankan perintah di shell baru yang terpisah
        subprocess.Popen(command_to_run, shell=True)
        rospy.loginfo("[SaverNode] Berhasil membuka terminal baru untuk map_saver.")
    except Exception as e:
        rospy.logerr(f"[SaverNode] Gagal membuka terminal baru: {e}")

def main():
    rospy.init_node('saver_node', anonymous=True)
    rospy.Subscriber('/save_map_command', String, save_map_callback)
    rospy.loginfo("SaverNode (Pekerja Penyimpan Peta) siap menerima perintah.")
    rospy.spin()

if __name__ == '__main__':
    main()
