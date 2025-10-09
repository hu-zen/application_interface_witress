# File: manager.py (dengan Mode Navigasi yang Benar)

import subprocess
import time
import os
import signal
import rospkg

class RosManager:
    def __init__(self, status_callback):
        # Proses yang sedang berjalan
        self.roscore_process = None
        self.controller_process = None
        self.mapping_process = None
        self.navigation_process = None # Tambahan untuk navigasi
        
        # Status flag
        self.is_controller_running = False
        self.is_mapping_running = False
        self.is_navigation_running = False # Tambahan untuk navigasi
        
        self.status_callback = status_callback
        self.rospack = rospkg.RosPack()
        
        self.current_map_name = None
        
        self.start_roscore()
        print("INFO: RosManager siap.")

    def start_roscore(self):
        # ... (Tidak ada perubahan di sini)
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
        # ... (Tidak ada perubahan di sini)
        if process and process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
                print(f"INFO: Grup proses '{name}' berhasil dihentikan.")
            except (ProcessLookupError, subprocess.TimeoutExpired, OSError):
                print(f"WARN: Gagal menghentikan '{name}' dengan normal.")
        return None

    # --- FUNGSI CONTROLLER & MAPPING (TIDAK BERUBAH) ---
    def start_controller(self):
        # ... (Tidak ada perubahan)
        if not self.is_controller_running:
            command = "roslaunch my_robot_pkg controller.launch"
            self.controller_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.is_controller_running = True
            print("INFO: Mode Controller dimulai.")
            return "Status: AKTIF"
        return "Status: Sudah Aktif"

    def stop_controller(self):
        # ... (Tidak ada perubahan)
        if self.is_controller_running:
            self.controller_process = self._stop_process_group(self.controller_process, "Controller")
            self.is_controller_running = False
        return "Status: DIMATIKAN"

    def start_mapping(self, map_name):
        # ... (Tidak ada perubahan)
        if not self.is_mapping_running:
            self.current_map_name = map_name
            command = "roslaunch autonomus_mobile_robot mapping.launch"
            self.mapping_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
            self.is_mapping_running = True
            print(f"INFO: Mode Mapping dimulai untuk peta '{self.current_map_name}'.")
            self.start_controller()
            return "Mode Pemetaan AKTIF.\nSilakan gerakkan robot."
        return "Status: Mapping Sudah Aktif"

    def stop_mapping(self):
        # ... (Tidak ada perubahan)
        if self.is_mapping_running:
            self._save_map_on_exit()
            self.mapping_process = self._stop_process_group(self.mapping_process, "Mapping")
            self.is_mapping_running = False
            self.stop_controller()
            self.current_map_name = None
        return "Status: DIMATIKAN"

    def _save_map_on_exit(self):
        # ... (Tidak ada perubahan)
        if not self.current_map_name: return
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
            map_save_path = os.path.join(pkg_path, 'maps', self.current_map_name)
            command = f"rosrun map_server map_saver -f {map_save_path}"
            print(f"INFO: Menyimpan peta otomatis ke '{map_save_path}'...")
            subprocess.run(command, shell=True, check=True, timeout=15)
            print("INFO: Peta berhasil disimpan!")
        except Exception as e:
            print(f"ERROR: Gagal menyimpan peta saat keluar: {e}")

    # --- FUNGSI NAVIGASI YANG DIPERBAIKI ---
    def get_map_list(self):
        """Mencari semua file .yaml di direktori peta."""
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
            maps_dir = os.path.join(pkg_path, 'maps')
            if not os.path.isdir(maps_dir):
                print(f"WARN: Direktori '{maps_dir}' tidak ditemukan.")
                return []
            
            map_files = [f.replace('.yaml', '') for f in os.listdir(maps_dir) if f.endswith('.yaml')]
            print(f"INFO: Peta ditemukan: {map_files}")
            return sorted(map_files) # Urutkan agar lebih rapi
        except rospkg.ResourceNotFound:
            print("ERROR: Paket 'autonomus_mobile_robot' tidak ditemukan.")
            return []

    def start_navigation(self, map_name):
        """Memulai proses navigasi dengan peta yang dipilih."""
        if self.is_navigation_running:
            return "Status: Navigasi sudah berjalan."
            
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
            map_file_path = os.path.join(pkg_path, 'maps', f"{map_name}.yaml")

            if not os.path.exists(map_file_path):
                return f"GAGAL: File peta '{map_name}.yaml' tidak ditemukan."

            # ===== PERUBAHAN DI SINI =====
            # Mengganti 'amcl_nav.launch' menjadi 'navigation.launch' sesuai informasi Anda
            command = f"roslaunch autonomus_mobile_robot navigation.launch map_file:={map_file_path}"
            
            self.navigation_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
            self.is_navigation_running = True
            print(f"INFO: Mode Navigasi dimulai dengan perintah: {command}")
            return f"Navigasi Aktif dengan Peta:\n{map_name}"
        except Exception as e:
            print(f"FATAL: Gagal menjalankan launch file navigasi: {e}")
            return f"GAGAL memulai navigasi!\nError: {e}"

    def stop_navigation(self):
        """Menghentikan proses navigasi."""
        if self.is_navigation_running:
            self.navigation_process = self._stop_process_group(self.navigation_process, "Navigation")
            self.is_navigation_running = False
        return "Status: Navigasi Dimatikan"

    def shutdown(self):
        print("INFO: Shutdown dipanggil, menghentikan semua proses...")
        self.stop_mapping()
        self.stop_controller()
        self.stop_navigation()
        
        if self.roscore_process:
            self.roscore_process = self._stop_process_group(self.roscore_process, "roscore")
