# File: manager.py (Versi Final dengan Save di Terminal Baru)

import subprocess
import threading
import time
import os
import signal
import rospkg

class RosManager:
    def __init__(self, status_callback):
        self.roscore_process = None
        self.controller_process = None
        self.mapping_process = None
        
        self.is_controller_running = False
        self.is_mapping_running = False
        
        self.status_callback = status_callback
        self.rospack = rospkg.RosPack()
        
        roscore_thread = threading.Thread(target=self.start_roscore)
        roscore_thread.daemon = True
        roscore_thread.start()
        
        self.monitor_thread = threading.Thread(target=self._monitor_processes)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        print("INFO: RosManager siap.")

    def start_roscore(self):
        if self.roscore_process is None:
            print("INFO: [THREAD] Memulai roscore...")
            try:
                self.roscore_process = subprocess.Popen("roscore", preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(3) 
                print("INFO: [THREAD] roscore seharusnya sudah berjalan.")
            except Exception as e:
                print(f"FATAL: Gagal memulai roscore: {e}")
                self.status_callback("main_menu", "status_label", "FATAL! Gagal memulai roscore.")

    def _monitor_processes(self):
        while True:
            if self.is_controller_running and self.controller_process and self.controller_process.poll() is not None:
                self.is_controller_running = False; self.controller_process = None
                self.status_callback("controller", "controller_status_label", "Status: Gagal! Proses berhenti.")
            
            if self.is_mapping_running and self.mapping_process and self.mapping_process.poll() is not None:
                self.is_mapping_running = False; self.mapping_process = None
                self.status_callback("mapping", "mapping_status_label", "Status: Gagal! Mapping berhenti.")
            
            time.sleep(1)

    # --- FUNGSI-FUNGSI MODE ---
    def start_controller(self):
        if not self.is_controller_running:
            command = "roslaunch my_robot_pkg controller.launch"
            self.controller_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
            self.is_controller_running = True
            return "Status: AKTIF"
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
            self.is_mapping_running = True
            self.start_controller()
            return "Mode Pemetaan AKTIF.\nController juga aktif."
        return "Status: Mapping Sudah Aktif"

    def stop_mapping(self):
        if self.is_mapping_running and self.mapping_process:
            os.killpg(os.getpgid(self.mapping_process.pid), signal.SIGTERM); self.mapping_process.wait()
            self.mapping_process = None; self.is_mapping_running = False
            self.stop_controller()
            return "Status: DIMATIKAN"
        return "Status: Memang tidak aktif"

    # ===== PERUBAHAN UTAMA UNTUK MENYIMPAN PETA =====
    def save_map(self, map_name):
        """Membuka terminal baru untuk menjalankan map_saver secara terpisah."""
        if not map_name:
            print("ERROR: Nama peta tidak boleh kosong.")
            self.status_callback("mapping", "mapping_status_label", "GAGAL: Nama peta kosong.")
            return False # Kembalikan False jika gagal

        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
        except rospkg.ResourceNotFound:
            print("ERROR: Paket 'autonomus_mobile_robot' tidak ditemukan.")
            self.status_callback("mapping", "mapping_status_label", "GAGAL: Paket Peta tidak ditemukan.")
            return False

        map_save_path = f"{pkg_path}/maps/{map_name}"
        
        # Perintah ini akan membuka terminal baru, menyiapkan lingkungan ROS,
        # menjalankan map_saver, lalu menunggu 5 detik sebelum menutup.
        command_to_run = f"""
        gnome-terminal -- /bin/bash -c "source /opt/ros/noetic/setup.bash; \\
        source ~/catkin_ws/devel/setup.bash; \\
        echo 'Menyimpan peta ke {map_save_path}...'; \\
        rosrun map_server map_saver -f {map_save_path}; \\
        echo 'Perintah Selesai. Terminal akan ditutup dalam 5 detik...'; \\
        sleep 5; exit"
        """
        
        try:
            # Jalankan perintah ini di latar belakang, GUI tidak perlu menunggu
            subprocess.Popen(command_to_run, shell=True)
            print(f"INFO: Membuka terminal baru untuk menyimpan peta '{map_name}'.")
            return True # Kembalikan True karena perintah berhasil dijalankan
        except Exception as e:
            print(f"ERROR: Gagal membuka terminal baru: {e}")
            self.status_callback("mapping", "mapping_status_label", "GAGAL membuka terminal.")
            return False
    # ============================================

    def shutdown(self):
        print("INFO: Shutdown dipanggil...")
        self.stop_mapping()
        if self.roscore_process:
            print("INFO: Menghentikan roscore...")
            os.killpg(os.getpgid(self.roscore_process.pid), signal.SIGTERM)
            self.roscore_process.wait()
