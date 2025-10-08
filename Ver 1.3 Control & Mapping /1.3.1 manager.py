# File: manager.py (Versi Perbaikan untuk Masalah Jendela Tidak Terbuka)

import subprocess
import threading
import time
import os
import signal
import rospy
from std_msgs.msg import String

class RosManager:
    def __init__(self, status_callback):
        if not rospy.core.is_initialized():
            rospy.init_node('waiterbot_gui_manager', anonymous=True)

        self.roscore_process = None
        self.controller_process = None
        self.mapping_process = None
        self.is_controller_running = False
        self.is_mapping_running = False
        self.status_callback = status_callback
        self.save_map_publisher = rospy.Publisher('/save_map_command', String, queue_size=10)
        
        # ===== PERUBAHAN UTAMA DI SINI =====
        # Jangan panggil start_roscore() langsung. Jalankan di thread terpisah.
        roscore_thread = threading.Thread(target=self.start_roscore)
        roscore_thread.daemon = True # Pastikan thread ini mati saat aplikasi utama ditutup
        roscore_thread.start()
        # ==================================
        
        self.monitor_thread = threading.Thread(target=self._monitor_processes)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        print("INFO: RosManager siap.")

    def start_roscore(self):
        """Fungsi ini sekarang berjalan di thread-nya sendiri, tidak akan memblokir GUI."""
        if self.roscore_process is None:
            print("INFO: [THREAD] Memulai roscore...")
            try:
                self.roscore_process = subprocess.Popen("roscore", preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(3) # Jeda ini sekarang aman karena ada di thread terpisah
                print("INFO: [THREAD] roscore seharusnya sudah berjalan.")
            except Exception as e:
                print(f"FATAL: [THREAD] Gagal memulai roscore: {e}")
                if self.status_callback:
                    self.status_callback("main_menu", "status_label", "FATAL! Gagal memulai roscore.")
        else:
            print("INFO: [THREAD] roscore sudah berjalan.")

    # ... (Semua fungsi lain dari _monitor_processes hingga shutdown tetap SAMA PERSIS seperti sebelumnya) ...
    def _monitor_processes(self):
        while True:
            if self.is_controller_running and self.controller_process and self.controller_process.poll() is not None:
                self.is_controller_running = False; self.controller_process = None
                if self.status_callback: self.status_callback("controller", "controller_status_label", "Status: Gagal!")
            if self.is_mapping_running and self.mapping_process and self.mapping_process.poll() is not None:
                self.is_mapping_running = False; self.mapping_process = None
                if self.status_callback: self.status_callback("mapping", "mapping_status_label", "Status: Gagal!")
            time.sleep(1)

    def start_controller(self):
        if not self.is_controller_running:
            command = "roslaunch my_robot_pkg controller.launch"
            self.controller_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
            self.is_controller_running = True; return "Status: AKTIF"
        return "Status: Sudah Aktif"

    def stop_controller(self):
        if self.is_controller_running and self.controller_process:
            os.killpg(os.getpgid(self.controller_process.pid), signal.SIGTERM); self.controller_process.wait()
            self.controller_process = None; self.is_controller_running = False
            return "Status: DIMATIKAN"
        return "Status: Memang tidak aktif"

    def start_mapping(self):
        if not self.is_mapping_running:
            command = "roslaunch autonomus_mobile_robot mapping.launch"
            self.mapping_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
            self.is_mapping_running = True; self.start_controller()
            return "Mode Pemetaan AKTIF.\nController juga aktif."
        return "Status: Mapping Sudah Aktif"

    def stop_mapping(self):
        if self.is_mapping_running and self.mapping_process:
            os.killpg(os.getpgid(self.mapping_process.pid), signal.SIGTERM); self.mapping_process.wait()
            self.mapping_process = None; self.is_mapping_running = False
            self.stop_controller()
            return "Status: DIMATIKAN"
        return "Status: Memang tidak aktif"

    def save_map(self, map_name):
        if not map_name:
            self.status_callback("mapping", "mapping_status_label", "GAGAL: Nama peta kosong.")
            return

        print(f"INFO: Mengirim perintah (via ROS Topic) untuk menyimpan peta '{map_name}'...")
        self.save_map_publisher.publish(map_name)
        self.status_callback("mapping", "mapping_status_label", f"Perintah simpan '{map_name}'\ntelah dikirim.")
        
    def shutdown(self):
        print("INFO: Shutdown dipanggil...")
        self.stop_mapping()
        if self.roscore_process:
            print("INFO: Menghentikan roscore...")
            os.killpg(os.getpgid(self.roscore_process.pid), signal.SIGTERM)
            self.roscore_process.wait()
