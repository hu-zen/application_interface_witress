# File: manager.py (Versi dengan Mode Controller + Mapping)

import subprocess
import threading
import time
import os
import signal
import rospkg # Kita butuh ini untuk menyimpan peta di lokasi yang benar

class RosManager:
    def __init__(self, status_callback):
        self.roscore_process = None
        self.controller_process = None
        self.mapping_process = None # <-- BARU: Proses untuk mapping
        
        self.is_controller_running = False
        self.is_mapping_running = False # <-- BARU: Status untuk mapping
        
        self.status_callback = status_callback
        self.rospack = rospkg.RosPack()
        
        self.start_roscore()
        
        self.monitor_thread = threading.Thread(target=self._monitor_processes)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        print("INFO: RosManager siap.")

    def start_roscore(self):
        if self.roscore_process is None:
            print("INFO: Memulai roscore di latar belakang...")
            try:
                self.roscore_process = subprocess.Popen("roscore", preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(3) 
                print("INFO: roscore seharusnya sudah berjalan.")
            except Exception as e:
                print(f"FATAL: Gagal memulai roscore: {e}")
                self.status_callback("main", "status_label", "FATAL! Gagal memulai roscore.")

    def _monitor_processes(self):
        """Sekarang memantau semua proses mode."""
        while True:
            # Pantau proses controller
            if self.is_controller_running and self.controller_process:
                if self.controller_process.poll() is not None:
                    print("ERROR: Proses controller.launch berhenti tak terduga!")
                    self.is_controller_running = False
                    self.controller_process = None
                    self.status_callback("controller", "status_label", "Status: Gagal! Proses berhenti.")
            
            # BARU: Pantau proses mapping
            if self.is_mapping_running and self.mapping_process:
                if self.mapping_process.poll() is not None:
                    print("ERROR: Proses mapping.launch berhenti tak terduga!")
                    self.is_mapping_running = False
                    self.mapping_process = None
                    self.status_callback("mapping", "status_label", "Status: Gagal! Proses berhenti.")
            
            time.sleep(1)

    # --- FUNGSI UNTUK MODE CONTROLLER ---
    def start_controller(self):
        if not self.is_controller_running:
            command = "roslaunch my_robot_pkg controller.launch"
            self.controller_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
            self.is_controller_running = True
            print("INFO: Mode Controller dimulai.")
            return "Status: AKTIF"
        return "Status: Sudah Aktif"

    def stop_controller(self):
        if self.is_controller_running and self.controller_process:
            os.killpg(os.getpgid(self.controller_process.pid), signal.SIGTERM)
            self.controller_process.wait()
            self.controller_process = None
            self.is_controller_running = False
            print("INFO: Mode Controller dihentikan.")
            return "Status: DIMATIKAN"
        return "Status: Memang tidak aktif"

    # --- FUNGSI BARU UNTUK MODE MAPPING ---
    def start_mapping(self):
        if not self.is_mapping_running:
            # Pastikan nama launch file ini benar
            command = "roslaunch autonomus_mobile_robot mapping.launch"
            self.mapping_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
            self.is_mapping_running = True
            print("INFO: Mode Mapping dimulai.")
            return "Mode Pemetaan AKTIF.\nGunakan joystick & RViz."
        return "Status: Mapping Sudah Aktif"

    def stop_mapping(self):
        if self.is_mapping_running and self.mapping_process:
            os.killpg(os.getpgid(self.mapping_process.pid), signal.SIGTERM)
            self.mapping_process.wait()
            self.mapping_process = None
            self.is_mapping_running = False
            print("INFO: Mode Mapping dihentikan.")
            return "Status: DIMATIKAN"
        return "Status: Memang tidak aktif"

    def save_map(self, map_name):
        """Fungsi baru untuk menyimpan peta."""
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
        except rospkg.ResourceNotFound:
            print("ERROR: Paket 'autonomus_mobile_robot' tidak ditemukan.")
            return False
            
        if not map_name:
            print("ERROR: Nama peta tidak boleh kosong.")
            return False

        map_save_path = f"{pkg_path}/maps/{map_name}"
        command = f"rosrun map_server map_saver -f {map_save_path}"
        
        print(f"INFO: Menyimpan peta ke {map_save_path}...")
        try:
            # Jalankan perintah simpan peta
            subprocess.run(command, shell=True, check=True, timeout=10)
            print("INFO: Peta berhasil disimpan!")
            return True
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            print(f"ERROR: Gagal menyimpan peta: {e}")
            return False

    def shutdown(self):
        print("INFO: Shutdown dipanggil, menghentikan semua proses...")
        self.stop_controller()
        self.stop_mapping() # <-- BARU: Hentikan mapping saat shutdown
        
        if self.roscore_process:
            print("INFO: Menghentikan roscore...")
            os.killpg(os.getpgid(self.roscore_process.pid), signal.SIGTERM)
            self.roscore_process.wait()
