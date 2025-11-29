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

# [PERBAIKAN 1]: Import Twist untuk komunikasi langsung ke robot
try:
    import rospy
    import tf
    from tf.transformations import euler_from_quaternion
    from geometry_msgs.msg import Twist # <-- Penting untuk pengereman cepat
except ImportError:
    print("PERINGATAN: Pustaka ROS tidak lengkap. Fitur real-time non-aktif.")
    rospy = None
    tf = None

class RosPoseListener(threading.Thread):
    def __init__(self):
        super(RosPoseListener, self).__init__()
        self.daemon = True
        self.listener = None
        self.robot_pose = None
        self._stop_event = threading.Event()
        self._run_event = threading.Event()

    def run(self):
        if not rospy or not tf: return
        if not rospy.core.is_initialized():
            rospy.init_node('kivy_ros_manager', anonymous=True, disable_signals=True)
        
        self.listener = tf.TransformListener()
        rate = rospy.Rate(10.0)

        while not self._stop_event.is_set():
            self._run_event.wait()
            if self._stop_event.is_set(): break
            try:
                (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                _, _, yaw = euler_from_quaternion(rot)
                self.robot_pose = {'x': trans[0], 'y': trans[1], 'yaw': yaw}
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.robot_pose = None
                continue
            finally:
                rate.sleep()

    def start_listening(self):
        self._run_event.set()

    def stop_listening(self):
        self._run_event.clear()
        self.robot_pose = None

    def stop_thread(self):
        self._stop_event.set()
        self._run_event.set()

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
        
        # [PERBAIKAN 2]: Variabel untuk Publisher Pengereman
        self.cmd_vel_pub = None 
        
        self.start_roscore_if_needed()
        self._init_ros_node()
        print("INFO: RosManager siap.")

    def start_roscore_if_needed(self):
        try:
            subprocess.check_output(["pidof", "roscore"])
        except subprocess.CalledProcessError:
            print("INFO: roscore belum berjalan, memulai di latar belakang...")
            try:
                self.roscore_process = subprocess.Popen("roscore", preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(4)
            except Exception as e:
                print(f"FATAL: Gagal memulai roscore: {e}")

    def _init_ros_node(self):
        if not rospy: return
        try:
            if not rospy.core.is_initialized():
                rospy.init_node('kivy_ros_manager', anonymous=True, disable_signals=True)
            
            # [PERBAIKAN 3]: Inisialisasi Publisher Langsung (Sangat Cepat)
            # Ini membuat jalur khusus untuk mengirim perintah STOP tanpa delay terminal
            self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            
            self.pose_listener = RosPoseListener()
            self.pose_listener.start()
            print("INFO: Node ROS & Publisher siap.")
        except Exception as e:
            print(f"FATAL: Gagal menginisialisasi node ROS: {e}")
            self.pose_listener = None
            self.cmd_vel_pub = None

    def _stop_process_group(self, process, name):
        if process and process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                # Tunggu sebentar untuk memastikan proses benar-benar mati
                # sebelum kita mengirim stop command terakhir
                try:
                    process.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    # Jika bandel, paksa bunuh (SIGKILL)
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except (ProcessLookupError, OSError):
                pass
        return None
    
    # [PERBAIKAN 4 - KRUSIAL]: Fungsi Pengereman Instan
    def _send_stop_command(self):
        print("INFO: Mengirim perintah STOP.")
        
        # OPSI A: Gunakan jalur cepat (Publisher Python)
        if rospy and self.cmd_vel_pub:
            stop_msg = Twist() # Default x=0, y=0, z=0
            # Kirim beruntun 10x sangat cepat untuk menimpa perintah joystick
            # Ini hanya butuh waktu ~0.05 detik total
            for _ in range(10):
                self.cmd_vel_pub.publish(stop_msg)
                time.sleep(0.01) 
        
        # OPSI B: Fallback jika publisher gagal (Jalur Lambat via Terminal)
        else:
            stop_cmd = 'rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"'
            subprocess.run(stop_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # Batalkan Goal Navigasi (jika ada)
        cancel_cmd = 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'
        subprocess.Popen(cancel_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # ---------------------------------------------------------
    # CONTROLLER (JOYSTICK)
    # ---------------------------------------------------------
    def start_controller(self):
        if not self.is_controller_running:
            command = "roslaunch my_robot_pkg controller.launch"
            self.controller_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.is_controller_running = True
            return "Status: AKTIF"
        return "Status: Sudah Aktif"

    def stop_controller(self):
        if self.is_controller_running:
            # [STRATEGI BARU]
            # 1. Matikan proses joystick DULUAN (sampai benar-benar mati/kill)
            self.controller_process = self._stop_process_group(self.controller_process, "Controller")
            self.is_controller_running = False
            
            # 2. SEGERA banjiri robot dengan perintah diam
            # Karena proses joystick sudah mati di langkah 1, tidak ada yang melawan perintah stop ini.
            self._send_stop_command()
        return "Status: DIMATIKAN"

    # ---------------------------------------------------------
    # NAVIGATION
    # ---------------------------------------------------------
    def start_navigation(self, map_name):
        if not self.is_navigation_running:
            try:
                self.current_map_name = map_name
                pkg_path = self.rospack.get_path('autonomus_mobile_robot')
                map_file_path = os.path.join(pkg_path, 'maps', f"{map_name}.yaml")
                
                command = f"roslaunch autonomus_mobile_robot gui_navigation.launch map_file:={map_file_path}"
                self.navigation_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
                self.is_navigation_running = True
                
                # Nyalakan joystick untuk override manual
                self.start_controller()
                
                if self.pose_listener:
                    time.sleep(5) 
                    self.pose_listener.start_listening()

                return f"Navigasi dengan peta\n'{map_name}' AKTIF"
            except Exception as e:
                print(f"FATAL: Gagal menjalankan navigasi: {e}")
                return f"GAGAL memulai navigasi!\nError: {e}"
        return "Status: Navigasi Sudah Aktif"
        
    def stop_navigation(self):
        if self.is_navigation_running:
            if self.pose_listener:
                self.pose_listener.stop_listening()
            
            # Matikan Joystick & Navigasi dulu
            self.stop_controller() 
            self.navigation_process = self._stop_process_group(self.navigation_process, "Navigation")
            self.is_navigation_running = False
            
            # Paksa Stop lagi untuk memastikan
            self._send_stop_command()
            
            self.current_map_name = None
            self.map_metadata = None
        return "Status: DIMATIKAN"

    # ---------------------------------------------------------
    # MAPPING
    # ---------------------------------------------------------
    def start_mapping(self, map_name):
        if not self.is_mapping_running:
            self.current_map_name = map_name
            command = "roslaunch autonomus_mobile_robot mapping.launch"
            try:
                self.mapping_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
                self.is_mapping_running = True
                self.start_controller()
                return "Mode Pemetaan AKTIF.\nSilakan gerakkan robot."
            except Exception as e:
                print(f"FATAL: Gagal menjalankan mapping: {e}"); return f"GAGAL memulai mapping!\nError: {e}"
        return "Status: Mapping Sudah Aktif"

    def stop_mapping(self):
        if self.is_mapping_running:
            self._save_map_on_exit()
            self.stop_controller()
            self.mapping_process = self._stop_process_group(self.mapping_process, "Mapping")
            self.is_mapping_running = False
            self._send_stop_command()
            self.current_map_name = None
        return "Status: DIMATIKAN"
        
    def cancel_mapping(self):
        if self.is_mapping_running:
            self.stop_controller()
            self.mapping_process = self._stop_process_group(self.mapping_process, "Mapping")
            self.is_mapping_running = False
            self._send_stop_command()
            self.current_map_name = None
        return "Status: DIBATALKAN"

    # --- FUNGSI UTILITAS LAINNYA ---
    def _save_map_on_exit(self):
        if not self.current_map_name: return
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
            map_save_path = os.path.join(pkg_path, 'maps', self.current_map_name)
            command = f"rosrun map_server map_saver -f {map_save_path}"
            subprocess.run(command, shell=True, check=True, timeout=15, capture_output=True, text=True)
            print("INFO: Peta berhasil disimpan!")
        except Exception as e:
            print(f"ERROR: Gagal menyimpan peta saat keluar: {e}")

    def shutdown(self):
        print("INFO: Shutdown dipanggil...")
        if self.pose_listener:
            self.pose_listener.stop_thread()
            self.pose_listener.join()
        
        self.stop_mapping()
        self.stop_navigation()
        self.stop_controller() 
        self._send_stop_command()
        
        if self.roscore_process:
            self._stop_process_group(self.roscore_process, "roscore")
    
    def get_robot_pose(self):
        if self.pose_listener:
            return self.pose_listener.get_pose()
        return None

    def get_available_maps(self):
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
            maps_dir = os.path.join(pkg_path, 'maps')
            map_files = glob.glob(os.path.join(maps_dir, '*.yaml'))
            return [os.path.splitext(os.path.basename(f))[0] for f in map_files]
        except Exception:
            return []

    def get_map_image_path(self, map_name):
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
            path = os.path.join(pkg_path, 'maps', f"{map_name}.pgm")
            return path if os.path.exists(path) else None
        except Exception:
            return None

    def load_map_metadata(self, map_name):
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
            with open(os.path.join(pkg_path, 'maps', f"{map_name}.yaml"), 'r') as f:
                self.map_metadata = yaml.safe_load(f)
        except Exception:
            self.map_metadata = None
