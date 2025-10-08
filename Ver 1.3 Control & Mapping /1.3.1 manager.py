# File: manager.py (Versi dengan Alur Kerja Mapping Baru)

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
        
        # ===== PERUBAHAN 1: Tambahkan variabel untuk menyimpan nama peta saat ini =====
        self.current_map_name = None
        
        self.status_callback = status_callback
        self.save_map_publisher = rospy.Publisher('/save_map_command', String, queue_size=10)
        
        roscore_thread = threading.Thread(target=self.start_roscore)
        roscore_thread.daemon = True
        roscore_thread.start()
        
        self.monitor_thread = threading.Thread(target=self._monitor_processes)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        print("INFO: RosManager siap.")

    # ... (start_roscore dan _monitor_processes tidak berubah) ...
    def start_roscore(self):
        if self.roscore_process is None:
            print("INFO: [THREAD] Memulai roscore...")
            try:
                self.roscore_process = subprocess.Popen("roscore", preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(3)
                print("INFO: [THREAD] roscore seharusnya sudah berjalan.")
            except Exception as e:
                print(f"FATAL: Gagal memulai roscore: {e}")
                if self.status_callback: self.status_callback("main_menu", "status_label", "FATAL! Gagal memulai roscore.")
    def _monitor_processes(self):
        while True:
            if self.is_controller_running and self.controller_process and self.controller_process.poll() is not None:
                self.is_controller_running = False; self.controller_process = None
                if self.status_callback: self.status_callback("controller", "controller_status_label", "Status: Gagal!")
            if self.is_mapping_running and self.mapping_process and self.mapping_process.poll() is not None:
                self.is_mapping_running = False; self.mapping_process = None
                if self.status_callback: self.status_callback("mapping_in_progress", "mapping_status_label", "Status: Gagal!")
            time.sleep(1)

    # --- FUNGSI-FUNGSI MODE ---
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

    # ===== PERUBAHAN 2: start_mapping sekarang menerima dan menyimpan nama peta =====
    def start_mapping(self, map_name):
        if not self.is_mapping_running:
            self.current_map_name = map_name # Simpan nama peta
            
            command = "roslaunch autonomus_mobile_robot mapping.launch"
            self.mapping_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
            self.is_mapping_running = True
            
            self.start_controller()
            return f"Membuat peta: {self.current_map_name}\nController Aktif."
        return "Status: Mapping Sudah Aktif"

    # ===== PERUBAHAN 3: Fungsi stop_mapping digabung dengan save =====
    def finish_and_save_mapping(self):
        """Fungsi baru yang menyimpan lalu menghentikan proses."""
        if self.is_mapping_running:
            # 1. Kirim perintah untuk menyimpan peta dengan nama yang sudah disimpan
            if self.current_map_name:
                print(f"INFO: Mengirim perintah simpan untuk peta '{self.current_map_name}'...")
                self.save_map_publisher.publish(self.current_map_name)
                # Beri jeda singkat agar pesan terkirim
                time.sleep(1) 
            
            # 2. Hentikan proses mapping dan controller
            if self.mapping_process:
                os.killpg(os.getpgid(self.mapping_process.pid), signal.SIGTERM)
                self.mapping_process.wait()
                self.mapping_process = None
            
            self.stop_controller()
            
            # 3. Reset status
            self.is_mapping_running = False
            self.current_map_name = None
            
            print("INFO: Mode Mapping telah selesai dan disimpan.")
            return "Status: Selesai & Disimpan"
        return "Status: Tidak ada proses mapping yang berjalan."
        
    def cancel_mapping(self):
        """Fungsi untuk batal mapping tanpa menyimpan."""
        if self.is_mapping_running:
            if self.mapping_process:
                os.killpg(os.getpgid(self.mapping_process.pid), signal.SIGTERM)
                self.mapping_process.wait()
                self.mapping_process = None
            self.stop_controller()
            self.is_mapping_running = False
            self.current_map_name = None
            print("INFO: Mode Mapping dibatalkan.")

    def shutdown(self):
        print("INFO: Shutdown dipanggil...")
        self.cancel_mapping() # Panggil cancel untuk membersihkan mapping & controller
        if self.roscore_process:
            print("INFO: Menghentikan roscore...")
            os.killpg(os.getpgid(self.roscore_process.pid), signal.SIGTERM)
            self.roscore_process.wait()
