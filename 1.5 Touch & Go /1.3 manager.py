# File: manager.py (Versi Final Touch-to-Go)

import subprocess
import time
import os
import signal
import rospkg
import glob
import yaml
import threading

# ===== PERUBAHAN 1: Tambahkan import yang dibutuhkan =====
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
# =======================================================

class RosManager:
    def __init__(self, status_callback):
        # ===== PERUBAHAN 2: Inisialisasi Node ROS di awal =====
        if not rospy.core.is_initialized():
            rospy.init_node('waiterbot_gui_manager', anonymous=True)
            print("INFO: Node ROS 'waiterbot_gui_manager' berhasil diinisialisasi.")
        # =======================================================

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
        self.map_metadata = None
        
        # ===== PERUBAHAN 3: Inisialisasi publisher sebagai None =====
        self.goal_publisher = None
        
        roscore_thread = threading.Thread(target=self.start_roscore)
        roscore_thread.daemon = True
        roscore_thread.start()
        
        print("INFO: RosManager siap.")

    def start_roscore(self):
        try:
            subprocess.check_output(["pidof", "roscore"])
            print("INFO: roscore sudah berjalan.")
        except subprocess.CalledProcessError:
            print("INFO: [THREAD] Memulai roscore...")
            try:
                self.roscore_process = subprocess.Popen("roscore", preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(4)
                print("INFO: [THREAD] roscore seharusnya sudah aktif.")
            except Exception as e:
                print(f"FATAL: Gagal memulai roscore: {e}")

    def _stop_process_group(self, process, name):
        if process and process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM); process.wait(timeout=5)
                print(f"INFO: Grup proses '{name}' dihentikan.")
            except Exception: print(f"WARN: Gagal menghentikan '{name}'.")
        return None

    # ... (Fungsi start/stop controller & mapping tidak berubah) ...
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
    def get_available_maps(self):
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
            maps_dir = os.path.join(pkg_path, 'maps')
            map_files = glob.glob(os.path.join(maps_dir, '*.yaml'))
            map_names = [os.path.splitext(os.path.basename(f))[0] for f in map_files]
            return map_names
        except Exception: return []

    def start_navigation(self, map_name):
        if not self.is_navigation_running:
            try:
                self.current_map_name = map_name
                pkg_path = self.rospack.get_path('autonomus_mobile_robot')
                map_file_path = os.path.join(pkg_path, 'maps', f"{map_name}.yaml")
                command = f"roslaunch autonomus_mobile_robot gui_navigation.launch map_file:={map_file_path}"
                self.navigation_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
                self.is_navigation_running = True
                self.start_controller()
                
                # ===== PERUBAHAN 4: Inisialisasi Publisher saat navigasi dimulai =====
                print("INFO: Menginisialisasi goal publisher...")
                self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
                time.sleep(1) # Beri sedikit waktu agar publisher siap
                print("INFO: Goal publisher berhasil diinisialisasi.")
                
                print(f"INFO: Mode Navigasi dimulai dengan peta '{map_name}'.")
                return f"Navigasi dengan peta\n'{map_name}' AKTIF"
            except Exception as e:
                return f"GAGAL memulai navigasi!\nError: {e}"
        return "Status: Navigasi Sudah Aktif"
        
    def stop_navigation(self):
        if self.is_navigation_running:
            self.navigation_process = self._stop_process_group(self.navigation_process, "Navigation")
            self.is_navigation_running = False
            self.stop_controller()
            self.current_map_name = None
            self.map_metadata = None
            # Matikan publisher saat mode berhenti
            if self.goal_publisher:
                self.goal_publisher.unregister()
                self.goal_publisher = None
                print("INFO: Goal publisher di-unregister.")
        return "Status: DIMATIKAN"
    
    def get_map_image_path(self, map_name):
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
            map_image_path = os.path.join(pkg_path, 'maps', f"{map_name}.pgm")
            if os.path.exists(map_image_path):
                return map_image_path
        except Exception as e:
            print(f"ERROR: Tidak dapat menemukan gambar peta: {e}")
        return None

    def load_map_metadata(self, map_name):
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
            map_yaml_path = os.path.join(pkg_path, 'maps', f"{map_name}.yaml")
            with open(map_yaml_path, 'r') as f:
                self.map_metadata = yaml.safe_load(f)
                print(f"INFO: Metadata untuk peta '{map_name}' dimuat.")
        except Exception as e:
            print(f"ERROR: Gagal memuat metadata peta: {e}")
            self.map_metadata = None

    # ===== PERUBAHAN 5: Gunakan rospy.Publisher yang sudah dibuat =====
    def send_goal_from_pixel(self, touch_x, touch_y, image_pixel_width, image_pixel_height):
        if not self.map_metadata or self.goal_publisher is None:
            print("ERROR: Metadata peta atau publisher belum siap.")
            self.status_callback("navigation", "navigation_status_label", "Error: Peta/Publisher belum siap.")
            return

        resolution = self.map_metadata['resolution']
        origin_x = self.map_metadata['origin'][0]
        origin_y = self.map_metadata['origin'][1]
        
        # Balik sumbu Y karena Kivy (0,0) di kiri bawah, PGM (0,0) di kiri atas
        pixel_y_reversed = image_pixel_height - touch_y
        
        map_x = (touch_x * resolution) + origin_x
        map_y = (pixel_y_reversed * resolution) + origin_y
        
        print(f"INFO: Sentuhan di ({touch_x:.2f}, {touch_y:.2f}) -> Goal ROS ({map_x:.2f}, {map_y:.2f})")

        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = map_x
        goal_msg.pose.position.y = map_y
        
        # Orientasi default (menghadap ke depan)
        q = quaternion_from_euler(0, 0, 0)
        goal_msg.pose.orientation.x = q[0]
        goal_msg.pose.orientation.y = q[1]
        goal_msg.pose.orientation.z = q[2]
        goal_msg.pose.orientation.w = q[3]

        self.goal_publisher.publish(goal_msg)
        print("INFO: Perintah GOAL dikirim ke /move_base_simple/goal")
    # ================================================================
            
    def shutdown(self):
        print("INFO: Shutdown dipanggil...")
        self.stop_mapping()
        self.stop_navigation()
        self.stop_controller()
        if self.roscore_process:
            self.roscore_process = self._stop_process_group(self.roscore_process, "roscore")
