# File: manager.py (Versi dengan Konversi Peta ke Gambar)

import subprocess
import time
import os
import signal
import rospkg
import glob
import rospy
from nav_msgs.msg import OccupancyGrid # <-- BARU: Untuk membaca data peta
from geometry_msgs.msg import PoseStamped # <-- BARU: Untuk mengirim goal nanti
import numpy as np # <-- BARU: Untuk memproses data peta
from PIL import Image # <-- BARU: Untuk membuat file gambar

class RosManager:
    def __init__(self, status_callback):
        if not rospy.core.is_initialized():
            rospy.init_node('waiterbot_gui_manager', anonymous=True)

        self.roscore_process = None
        self.controller_process = None
        self.mapping_process = None
        self.navigation_process = None

        self.is_controller_running = False
        self.is_mapping_running = False
        self.is_navigation_running = False
        
        self.status_callback = status_callback
        self.rospack = rospkg.RosPack()
        
        self.current_map_name = None
        self.map_info = None # <-- BARU: Untuk menyimpan metadata peta (resolusi, origin)
        self.map_filepath = os.path.join(os.path.expanduser('~'), '.ros', 'map_image.png') # Path gambar peta

        # ===== PERUBAHAN 1: Tambahkan Subscriber & Publisher =====
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        self.start_roscore()
        print("INFO: RosManager siap.")

    # ... (start_roscore, _stop_process_group, mode controller & mapping tidak berubah) ...
    def start_roscore(self):
        try:
            subprocess.check_output(["pidof", "roscore"])
        except subprocess.CalledProcessError:
            try:
                self.roscore_process = subprocess.Popen("roscore", preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(4)
            except Exception as e: print(f"FATAL: {e}")
    def _stop_process_group(self, process, name):
        if process and process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM); process.wait(timeout=5)
                print(f"INFO: Grup proses '{name}' dihentikan.")
            except Exception: print(f"WARN: Gagal menghentikan '{name}'.")
        return None
    def start_controller(self):
        if not self.is_controller_running:
            self.controller_process = subprocess.Popen("roslaunch my_robot_pkg controller.launch", shell=True, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.is_controller_running = True; return "Status: AKTIF"
        return "Status: Sudah Aktif"
    def stop_controller(self):
        if self.is_controller_running:
            self.controller_process = self._stop_process_group(self.controller_process, "Controller"); self.is_controller_running = False
        return "Status: DIMATIKAN"
    def start_mapping(self, map_name):
        if not self.is_mapping_running:
            self.current_map_name = map_name
            self.mapping_process = subprocess.Popen("roslaunch autonomus_mobile_robot mapping.launch", shell=True, preexec_fn=os.setsid)
            self.is_mapping_running = True; self.start_controller()
            return "Mode Pemetaan AKTIF.\nSilakan gerakkan robot."
        return "Status: Mapping Sudah Aktif"
    def stop_mapping(self):
        if self.is_mapping_running:
            self._save_map_on_exit()
            self.mapping_process = self._stop_process_group(self.mapping_process, "Mapping"); self.is_mapping_running = False
            self.stop_controller(); self.current_map_name = None
        return "Status: DIMATIKAN"
    def _save_map_on_exit(self):
        if not self.current_map_name: return
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
            map_save_path = os.path.join(pkg_path, 'maps', self.current_map_name)
            command = f"rosrun map_server map_saver -f {map_save_path}"
            subprocess.run(command, shell=True, check=True, timeout=15, capture_output=True, text=True)
            print(f"INFO: Peta '{self.current_map_name}' disimpan.")
        except Exception as e: print(f"ERROR: Gagal menyimpan peta: {e}")

    # ===== PERUBAHAN 2: Tambahkan fungsi map_callback =====
    def map_callback(self, msg):
        """Dipanggil setiap kali data peta baru diterima dari ROS."""
        print("INFO: Menerima data peta baru. Mengonversi ke gambar...")
        self.map_info = msg.info # Simpan metadata untuk konversi koordinat nanti
        
        # Ubah data peta (list) menjadi array numpy 2D
        map_array = np.array(msg.data, dtype=np.uint8).reshape((self.map_info.height, self.map_info.width))
        
        # Konversi nilai OccupancyGrid (0=bebas, 100=halangan, -1=tidak diketahui)
        # menjadi nilai grayscale (0-255)
        map_array[map_array == 0] = 254   # Area bebas -> hampir putih
        map_array[map_array == 100] = 0   # Halangan -> hitam
        map_array[map_array == -1] = 205  # Area tidak diketahui -> abu-abu
        
        # Buat gambar dari array
        img = Image.fromarray(map_array, 'L')
        # Peta dari ROS biasanya terbalik, jadi kita putar
        img = img.transpose(Image.FLIP_TOP_BOTTOM)
        
        # Simpan gambar ke file
        img.save(self.map_filepath)
        print(f"INFO: Peta berhasil disimpan sebagai gambar di {self.map_filepath}")
        
        # Beri tahu GUI bahwa gambar peta baru sudah siap
        self.status_callback("navigation", "update_map_image", self.map_filepath)

    # --- FUNGSI-FUNGSI NAVIGASI (start/stop tidak berubah) ---
    def get_available_maps(self):
        # ... (fungsi ini tidak berubah)
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
            maps_dir = os.path.join(pkg_path, 'maps')
            map_files = glob.glob(os.path.join(maps_dir, '*.yaml'))
            map_names = [os.path.splitext(os.path.basename(f))[0] for f in map_files]
            return map_names
        except Exception: return []

    def start_navigation(self, map_name):
        # ... (fungsi ini tidak berubah)
        if not self.is_navigation_running:
            try:
                pkg_path = self.rospack.get_path('autonomus_mobile_robot')
                map_file_path = os.path.join(pkg_path, 'maps', f"{map_name}.yaml")
                command = f"roslaunch autonomus_mobile_robot gui_navigation.launch map_file:={map_file_path}"
                self.navigation_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
                self.is_navigation_running = True
                self.start_controller()
                return f"Navigasi peta\n'{map_name}' AKTIF"
            except Exception as e:
                return f"GAGAL: {e}"
        return "Status: Navigasi Sudah Aktif"
        
    def stop_navigation(self):
        # ... (fungsi ini tidak berubah)
        if self.is_navigation_running:
            self.navigation_process = self._stop_process_group(self.navigation_process, "Navigation")
            self.is_navigation_running = False
            self.stop_controller()
        return "Status: DIMATIKAN"
    
    def shutdown(self):
        # ... (fungsi ini tidak berubah)
        self.stop_mapping()
        self.stop_controller()
        self.stop_navigation()
        if self.roscore_process:
            self.roscore_process = self._stop_process_group(self.roscore_process, "roscore")
