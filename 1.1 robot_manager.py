#!/usr/bin/env python3
import rospy
import subprocess
import os
import signal
from std_msgs.msg import String

# Variabel global untuk menyimpan proses yang sedang berjalan
robot_process = None

def stop_robot_launch():
    """Menghentikan proses launch yang sedang berjalan."""
    global robot_process
    if robot_process:
        rospy.loginfo("Menghentikan proses launch witress_bot...")
        # Menggunakan os.killpg untuk memastikan semua turunan proses berhenti
        os.killpg(os.getpgid(robot_process.pid), signal.SIGINT)
        robot_process.wait()
        robot_process = None
        rospy.loginfo("Proses berhasil dihentikan.")

def start_robot_launch(command_key):
    """Memulai launch file witress_bot berdasarkan kunci perintah."""
    global robot_process
    
    stop_robot_launch() # Hentikan proses lama sebelum memulai yang baru
    rospy.sleep(1)

    ros_env = os.environ.copy()
    command = []

    # Tentukan perintah berdasarkan pesan dari GUI
    if command_key == "start_controller":
        command = ["roslaunch", "my_robot_pkg", "controller.launch"]
    elif command_key == "start_mapping":
        command = ["roslaunch", "autonomus_mobile_robot", "mapping.launch"]
    else:
        rospy.logwarn(f"Perintah '{command_key}' tidak dikenali.")
        return

    try:
        rospy.loginfo(f"Manajer: Menjalankan perintah: {' '.join(command)}")
        robot_process = subprocess.Popen(command, env=ros_env, preexec_fn=os.setsid)
        rospy.loginfo("Perintah roslaunch telah dieksekusi.")
    except Exception as e:
        rospy.logerr(f"Gagal menjalankan roslaunch: {e}")

def main():
    rospy.init_node('robot_manager_node')
    rospy.Subscriber('/gui/robot_command', String, start_robot_launch)
    rospy.on_shutdown(stop_robot_launch) # Pastikan berhenti saat node dimatikan
    rospy.loginfo("Manajer Robot (witress_bot) siap.")
    rospy.spin()

if __name__ == '__main__':
    main()
