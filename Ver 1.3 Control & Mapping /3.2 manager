# File: manager.py (Solusi Final dengan Simpan Otomatis di Akhir)

import subprocess
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
        
        self.current_map_name = None
        
        self.start_roscore()
        print("INFO: RosManager siap.")

    def start_roscore(self):
        try:
            subprocess.check_output(["pidof", "roscore"])
            print("INFO: roscore sudah berjalan.")
        except subprocess.CalledProcessError:
            print("INFO: Memulai roscore di latar belakang...")
            try:
                self.roscore_process = subprocess.Popen("roscore", preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(4)
                print("INFO: roscore seharusnya sudah aktif.")
            except Exception as e:
                print(f"FATAL: Gagal memulai roscore: {e}")

    def _stop_process_group(self, process, name):
        if process and process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
                print(f"INFO: Grup proses '{name}' berhasil dihentikan.")
            except (ProcessLookupError, subprocess.TimeoutExpired, OSError):
                print(f"WARN: Gagal menghentikan '{name}' dengan normal.")
        return None

    def start_controller(self):
        if not self.is_controller_running:
            command = "roslaunch my_robot_pkg controller.launch"
            self.controller_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.is_controller_running = True
            print("INFO: Mode Controller dimulai.")
            return "Status: AKTIF"
        return "Status: Sudah Aktif"

    def stop_controller(self):
        if self.is_controller_running:
            self.controller_process = self._stop_process_group(self.controller_process, "Controller")
            self.is_controller_running = False
        return "Status: DIMATIKAN"

    def start_mapping(self, map_name):
        if not self.is_mapping_running:
            self.current_map_name = map_name
            command = "roslaunch autonomus_mobile_robot mapping.launch"
            try:
                self.mapping_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
                self.is_mapping_running = True
                print(f"INFO: Mode Mapping dimulai untuk peta '{self.current_map_name}'.")
                self.start_controller()
                return "Mode Pemetaan AKTIF.\nSilakan gerakkan robot."
            except Exception as e:
                print(f"FATAL: Gagal menjalankan roslaunch: {e}")
                return f"GAGAL memulai mapping!\nError: {e}"
        return "Status: Mapping Sudah Aktif"

    def stop_mapping(self):
        if self.is_mapping_running:
            self._save_map_on_exit()
            print("INFO: Menghentikan proses mapping...")
            self.mapping_process = self._stop_process_group(self.mapping_process, "Mapping")
            self.is_mapping_running = False
            self.stop_controller()
            self.current_map_name = None
        return "Status: DIMATIKAN"

    def _save_map_on_exit(self):
        if not self.current_map_name:
            print("WARN: Tidak ada nama peta, penyimpanan dilewati.")
            return
        
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
            map_save_path = os.path.join(pkg_path, 'maps', self.current_map_name)
            command = f"rosrun map_server map_saver -f {map_save_path}"
            print(f"INFO: Menyimpan peta secara otomatis ke '{map_save_path}'...")
            
            # Menjalankan map_saver. Ini akan sedikit memblokir, tapi aman karena
            # GUI sudah dalam proses keluar dari layar ini.
            result = subprocess.run(command, shell=True, check=True, timeout=15, capture_output=True, text=True)
            
            print("INFO: Peta berhasil disimpan!")
            print(result.stdout)
        except Exception as e:
            print(f"ERROR: Gagal menyimpan peta saat keluar: {e}")

    def shutdown(self):
        print("INFO: Shutdown dipanggil, menghentikan semua proses...")
        self.stop_mapping()
        self.stop_controller()
        if self.roscore_process:
            self.roscore_process = self._stop_process_group(self.roscore_process, "roscore")
