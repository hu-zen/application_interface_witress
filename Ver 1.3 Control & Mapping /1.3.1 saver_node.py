#!/usr/bin/env python3

import rospy
import subprocess
import os
import rospkg
from std_srvs.srv import Trigger, TriggerResponse

# Variabel global untuk menyimpan path paket
PKG_PATH = ""

def handle_save_map_request(req):
    """Callback yang dipanggil saat service diminta."""
    try:
        # Dapatkan timestamp untuk nama file default jika nama tidak diberikan
        import time
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        map_name = f"map_{timestamp}" # Nama default
        
        # Di masa depan, Anda bisa mengirim nama peta via service jika diperlukan
        # Untuk sekarang, kita gunakan nama dengan timestamp
        
        rospy.loginfo("Permintaan penyimpanan peta diterima.")
        
        if not PKG_PATH:
            rospy.logerr("Path paket 'autonomus_mobile_robot' tidak ditemukan.")
            return TriggerResponse(success=False, message="Package path not found.")

        # Bentuk path lengkap untuk menyimpan peta
        map_save_path = os.path.join(PKG_PATH, 'maps', map_name)
        command = f"rosrun map_server map_saver -f {map_save_path}"
        
        rospy.loginfo(f"Menjalankan perintah: {command}")
        
        # Jalankan perintah map_saver
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate(timeout=15) # Tunggu hingga selesai
        
        if process.returncode == 0:
            rospy.loginfo(f"Peta berhasil disimpan sebagai '{map_name}.yaml' dan '{map_name}.pgm'")
            return TriggerResponse(success=True, message=f"Peta '{map_name}' berhasil disimpan.")
        else:
            rospy.logerr(f"Gagal menyimpan peta. Error: {stderr.decode('utf-8')}")
            return TriggerResponse(success=False, message=f"Gagal menyimpan peta: {stderr.decode('utf-8')}")
            
    except Exception as e:
        rospy.logerr(f"Exception saat mencoba menyimpan peta: {str(e)}")
        return TriggerResponse(success=False, message=str(e))

def map_saver_server():
    global PKG_PATH
    rospy.init_node('map_saver_service_node')
    
    # Dapatkan path paket sekali saat node dimulai
    try:
        rospack = rospkg.RosPack()
        PKG_PATH = rospack.get_path('autonomus_mobile_robot')
    except rospkg.ResourceNotFound:
        rospy.logfatal("Paket 'autonomus_mobile_robot' tidak ditemukan. Pastikan workspace sudah di-source.")
        return

    # Buat ROS Service dengan nama 'save_map_service' dan tipe Trigger
    s = rospy.Service('save_map_service', Trigger, handle_save_map_request)
    rospy.loginfo("Service 'save_map_service' siap.")
    rospy.spin()

if __name__ == "__main__":
    map_saver_server()
