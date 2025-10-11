# File: manager.py

import subprocess
import time
import os
import signal
import rospkg
import glob
import yaml
import threading
import math

# --- Tambahan untuk Listener Posisi Robot ---
try:
    import rospy
    import tf
    from tf.transformations import euler_from_quaternion
except ImportError:
    print("FATAL: Pustaka 'rospy' atau 'tf' tidak ditemukan. Fungsionalitas posisi real-time tidak akan bekerja.")
    rospy = None
    tf = None
# -----------------------------------------


class RosPoseListener(threading.Thread):
    def __init__(self):
        super(RosPoseListener, self).__init__()
        self.daemon = True
        self.listener = None
        self.robot_pose = None
        self._stop_event = threading.Event()
        self._run_event = threading.Event() # Event untuk mengontrol loop utama

    def run(self):
        if not rospy or not tf:
            return

        print("INFO: Thread listener posisi robot dimulai.")
        self.listener = tf.TransformListener()
        rate = rospy.Rate(10.0) # 10 Hz

        while not self._stop_event.is_set():
            # Tunggu sampai event `run` diaktifkan
            self._run_event.wait() 
            
            # Jika event stop juga aktif, keluar dari loop
            if self._stop_event.is_set():
                break

            try:
                (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                _, _, yaw = euler_from_quaternion(rot)
                self.robot_pose = {'x': trans[0], 'y': trans[1], 'yaw': yaw}
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.robot_pose = None # Hapus pose jika tidak ditemukan
                continue
            finally:
                rate.sleep()

        print("INFO: Thread listener posisi robot dihentikan.")

    def start_listening(self):
        self._run_event.set() # Aktifkan loop

    def stop_listening(self):
        self._run_event.clear() # Jeda loop
        self.robot_pose = None

    def stop_thread(self):
        self._stop_event.set()
        self._run_event.set() # Pastikan thread tidak terjebak di `wait()`

    def get_pose(self):
        return self.robot_pose


class RosManager:
    def __init__(self, status_callback):
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

        self.pose_listener = None
        
        self.start_roscore()
        self._init_ros_node() # Inisialisasi node ROS sekali di awal
        print("INFO: RosManager siap.")

    def _init_ros_node(self):
        """Menginisialisasi node rospy hanya sekali."""
        if not rospy: return
        try:
            # Periksa apakah roscore berjalan sebelum init_node
            subprocess.check_output(["pidof", "roscore"])
            rospy.init_node('kivy_tf_listener', anonymous=True, disable_signals=True)
            print("INFO: Node ROS 'kivy_tf_listener' berhasil diinisialisasi.")
            # Buat thread listener sekarang, tapi jangan mulai loop-nya
            self.pose_listener = RosPoseListener()
            self.pose_listener.start()
        except (rospy.ROSInitException, subprocess.CalledProcessError) as e:
            print(f"WARN: Gagal menginisialisasi node ROS. Mungkin roscore belum siap. {e}")
            self.pose_listener = None
        except Exception as e:
            print(f"FATAL: Terjadi error tak terduga saat inisialisasi node ROS: {e}")
            self.pose_listener = None

    def _stop_process_group(self, process, name):
        if process and process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
                print(f"INFO: Grup proses '{name}' berhasil dihentikan.")
            except (ProcessLookupError, subprocess.TimeoutExpired, OSError):
                print(f"WARN: Gagal menghentikan '{name}' dengan normal.")
        return None

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

                # "Bangunkan" thread listener
                if self.pose_listener:
                    print("INFO: Mengaktifkan listener posisi robot.")
                    time.sleep(5)  # Beri waktu agar node ROS lain (seperti AMCL) siap
                    self.pose_listener.start_listening()
                
                print(f"INFO: Mode Navigasi dimulai dengan peta '{map_name}'.")
                return f"Navigasi dengan peta\\n'{map_name}' AKTIF"
            except Exception as e:
                print(f"FATAL: Gagal menjalankan navigasi: {e}")
                return f"GAGAL memulai navigasi!\\nError: {e}"
        return "Status: Navigasi Sudah Aktif"
        
    def stop_navigation(self):
        if self.is_navigation_running:
            # "Tidurkan" thread listener
            if self.pose_listener:
                print("INFO: Menjeda listener posisi robot.")
                self.pose_listener.stop_listening()

            self.navigation_process = self._stop_process_group(self.navigation_process, "Navigation")
            self.is_navigation_running = False
            self.stop_controller()
            self.current_map_name = None
            self.map_metadata = None
        return "Status: DIMATIKAN"
    
    def shutdown(self):
        print("INFO: Shutdown dipanggil, menghentikan semua proses...")
        self.stop_mapping()
        self.stop_navigation()
        self.stop_controller() 
        if self.roscore_process:
            self.roscore_process = self._stop_process_group(self.roscore_process, "roscore")
        # Hentikan thread listener secara permanen
        if self.pose_listener:
            self.pose_listener.stop_thread()
            self.pose_listener.join()
    
    # --- Sisa fungsi lainnya tidak berubah ---
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
            print("INFO: Mode Controller dihentikan.")
        return "Status: DIMATIKAN"

    def start_mapping(self, map_name):
        if not self.is_mapping_running:
            self.current_map_name = map_name
            command = "roslaunch autonomus_mobile_robot mapping.launch"
            try:
                self.mapping_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
                self.is_mapping_running = True
                print(f"INFO: Mode Mapping dimulai untuk '{self.current_map_name}'.")
                self.start_controller()
                return "Mode Pemetaan AKTIF.\\nSilakan gerakkan robot."
            except Exception as e:
                print(f"FATAL: Gagal menjalankan mapping: {e}"); return f"GAGAL memulai mapping!\\nError: {e}"
        return "Status: Mapping Sudah Aktif"

    def stop_mapping(self):
        if self.is_mapping_running:
            self._save_map_on_exit()
            self.mapping_process = self._stop_process_group(self.mapping_process, "Mapping")
            self.is_mapping_running = False
            self.stop_controller()
            self.current_map_name = None
        return "Status: DIMATIKAN"

    def _save_map_on_exit(self):
        if not self.current_map_name: return
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
            map_save_path = os.path.join(pkg_path, 'maps', self.current_map_name)
            command = f"rosrun map_server map_saver -f {map_save_path}"
            print(f"INFO: Menyimpan peta otomatis ke '{map_save_path}'...")
            result = subprocess.run(command, shell=True, check=True, timeout=15, capture_output=True, text=True)
            print("INFO: Peta berhasil disimpan!"); print(result.stdout)
        except Exception as e:
            print(f"ERROR: Gagal menyimpan peta saat keluar: {e}")

    def get_available_maps(self):
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
            maps_dir = os.path.join(pkg_path, 'maps')
            map_files = glob.glob(os.path.join(maps_dir, '*.yaml'))
            map_names = [os.path.splitext(os.path.basename(f))[0] for f in map_files]
            return map_names
        except Exception as e:
            print(f"ERROR: Gagal mencari peta: {e}")
            return []

    def get_robot_pose(self):
        if self.pose_listener:
            return self.pose_listener.get_pose()
        return None

    def get_map_image_path(self, map_name):
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
            map_image_path = os.path.join(pkg_path, 'maps', f"{map_name}.pgm")
            if os.path.exists(map_image_path):
                return map_image_path
        except Exception as e:
            print(f"ERROR: Tidak dapat menemukan file gambar untuk peta '{map_name}': {e}")
        return None

    def load_map_metadata(self, map_name):
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
            map_yaml_path = os.path.join(pkg_path, 'maps', f"{map_name}.yaml")
            with open(map_yaml_path, 'r') as f:
                self.map_metadata = yaml.safe_load(f)
                print(f"INFO: Metadata untuk peta '{map_name}' dimuat.")
        except Exception as e:
            print(f"ERROR: Gagal memuat metadata peta '{map_name}': {e}")
            self.map_metadata = None

    def send_goal_from_pixel(self, touch_x, touch_y, image_width, image_height):
        if not self.map_metadata:
            print("ERROR: Metadata peta belum dimuat. Tidak bisa mengirim goal.")
            return

        resolution = self.map_metadata['resolution']
        origin_x = self.map_metadata['origin'][0]
        origin_y = self.map_metadata['origin'][1]
        
        pixel_y_reversed = image_height - touch_y
        map_x = (touch_x * resolution) + origin_x
        map_y = (pixel_y_reversed * resolution) + origin_y
        
        print(f"INFO: Sentuhan di ({touch_x:.2f}, {touch_y:.2f}) -> Dikonversi ke Goal ROS ({map_x:.2f}, {map_y:.2f})")

        goal_msg_yaml = f"""header:
  stamp: now
  frame_id: "map"
pose:
  position:
    x: {map_x}
    y: {map_y}
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"""

        command = f'rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "{goal_msg_yaml}"'
        
        try:
            subprocess.Popen(command, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("INFO: Perintah GOAL dikirim ke /move_base_simple/goal")
        except Exception as e:
            print(f"ERROR: Gagal mengirim perintah goal: {e}")
