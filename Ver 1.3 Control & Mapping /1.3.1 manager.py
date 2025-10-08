# File: manager.py (Diperbaiki)

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
                self.status_callback("main_menu", "status_label", "FATAL! Gagal memulai roscore.") # Mengupdate screen menu utama jika perlu

    def _monitor_processes(self):
        while True:
            if self.is_controller_running and self.controller_process:
                if self.controller_process.poll() is not None:
                    print("ERROR: Proses controller.launch berhenti tak terduga!")
                    self.is_controller_running = False
                    self.controller_process = None
                    self.status_callback("controller", "status_label", "Status: Gagal! Proses berhenti.")
            
            if self.is_mapping_running and self.mapping_process:
                if self.mapping_process.poll() is not None:
                    print("ERROR: Proses mapping.launch berhenti tak terduga!")
                    self.is_mapping_running = False
                    self.mapping_process = None
                    self.status_callback("mapping", "mapping_status_label", "Status: Gagal! Mapping berhenti.")
            
            time.sleep(1)

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
            try:
                os.killpg(os.getpgid(self.controller_process.pid), signal.SIGTERM)
                self.controller_process.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass # Proses mungkin sudah mati
            finally:
                self.controller_process = None
                self.is_controller_running = False
                print("INFO: Mode Controller dihentikan.")
                return "Status: DIMATIKAN"
        return "Status: Memang tidak aktif"

    def start_mapping(self):
        if not self.is_mapping_running:
            command = "roslaunch autonomus_mobile_robot mapping.launch"
            self.mapping_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
            self.is_mapping_running = True
            print("INFO: Mode Mapping dimulai.")
            
            print("INFO: Menjalankan controller untuk mapping...")
            self.start_controller()
            
            return "Mode Pemetaan AKTIF.\nController juga aktif."
        return "Status: Mapping Sudah Aktif"

    def stop_mapping(self):
        if self.is_mapping_running and self.mapping_process:
            try:
                os.killpg(os.getpgid(self.mapping_process.pid), signal.SIGTERM)
                self.mapping_process.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
            finally:
                self.mapping_process = None
                self.is_mapping_running = False
                print("INFO: Mode Mapping dihentikan.")
            
            print("INFO: Menghentikan controller...")
            self.stop_controller()
            
            return "Status: DIMATIKAN"
        return "Status: Memang tidak aktif"

    # --- PERUBAHAN UTAMA DIMULAI DI SINI ---

    def save_map(self, map_name):
        """Memulai proses penyimpanan peta di thread terpisah."""
        if not map_name:
            self.status_callback("mapping", "mapping_status_label", "GAGAL: Nama peta tidak boleh kosong.")
            return

        # Umpan balik langsung ke user bahwa proses dimulai
        self.status_callback("mapping", "mapping_status_label", f"Menyimpan peta '{map_name}'...")
        
        # Buat dan jalankan thread untuk menyimpan peta
        save_thread = threading.Thread(target=self._execute_save_map, args=(map_name,))
        save_thread.daemon = True
        save_thread.start()

    def _execute_save_map(self, map_name):
        """Fungsi ini berjalan di background dan tidak akan memblokir GUI."""
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
        except rospkg.ResourceNotFound:
            print("ERROR: Paket 'autonomus_mobile_robot' tidak ditemukan.")
            self.status_callback("mapping", "mapping_status_label", "GAGAL: Paket ROS tidak ditemukan.")
            return

        map_save_path = os.path.join(pkg_path, 'maps', map_name)
        command = f"rosrun map_server map_saver -f {map_save_path}"
        
        print(f"INFO: Menjalankan perintah: {command}")
        try:
            # subprocess.run tetap blocking, tapi sekarang di dalam thread-nya sendiri
            subprocess.run(command, shell=True, check=True, timeout=15, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            print("INFO: Peta berhasil disimpan!")
            # Kirim status sukses ke GUI melalui callback
            self.status_callback("mapping", "mapping_status_label", f"Peta '{map_name}' BERHASIL disimpan!")
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            error_message = e.stderr.decode('utf-8') if hasattr(e, 'stderr') else str(e)
            print(f"ERROR: Gagal menyimpan peta: {error_message}")
            # Kirim status gagal ke GUI melalui callback
            self.status_callback("mapping", "mapping_status_label", f"GAGAL menyimpan peta.\nError: Cek terminal.")

    # --- PERUBAHAN UTAMA SELESAI ---

    def shutdown(self):
        print("INFO: Shutdown dipanggil, menghentikan semua proses...")
        self.stop_mapping() # stop_mapping sudah termasuk stop_controller
        
        if self.roscore_process:
            print("INFO: Menghentikan roscore...")
            try:
                os.killpg(os.getpgid(self.roscore_process.pid), signal.SIGTERM)
                self.roscore_process.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
