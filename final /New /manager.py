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

# Coba impor library ROS. Jika gagal, program tetap jalan tanpa fitur real-time.
try:
    import rospy
    import tf
    from tf.transformations import euler_from_quaternion
except ImportError:
    print("PERINGATAN: Pustaka 'rospy' atau 'tf' tidak ditemukan. Fungsionalitas posisi real-time tidak akan bekerja.")
    rospy = None
    tf = None

class RosPoseListener(threading.Thread):
    """Thread yang berjalan di latar belakang untuk mendengarkan posisi robot dari TF."""
    def __init__(self):
        super(RosPoseListener, self).__init__()
        self.daemon = True
        self.listener = None
        self.robot_pose = None
        self._stop_event = threading.Event()
        self._run_event = threading.Event()  # Event untuk menjeda/melanjutkan loop

    def run(self):
        if not rospy or not tf:
            return

        print("INFO (Thread): Listener posisi robot dimulai.")
        # Inisialisasi node di dalam thread jika belum ada
        if not rospy.core.is_initialized():
            rospy.init_node('kivy_pose_listener_thread', anonymous=True, disable_signals=True)
        
        self.listener = tf.TransformListener()
        rate = rospy.Rate(10.0)  # 10 Hz

        while not self._stop_event.is_set():
            self._run_event.wait()  # Tunggu di sini sampai diaktifkan
            
            if self._stop_event.is_set():
                break

            try:
                # Dapatkan transformasi antara frame peta dan frame dasar robot
                (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                
                # Konversi quaternion ke sudut Euler untuk mendapatkan rotasi (yaw)
                _, _, yaw = euler_from_quaternion(rot)
                
                # Simpan posisi (meter) dan orientasi (radian)
                self.robot_pose = {'x': trans[0], 'y': trans[1], 'yaw': yaw}
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.robot_pose = None  # Jika robot tidak terdeteksi, hapus posisinya
                continue
            finally:
                rate.sleep()
        print("INFO (Thread): Listener posisi robot dihentikan.")

    def start_listening(self):
        self._run_event.set()  # Aktifkan loop di dalam run()

    def stop_listening(self):
        self._run_event.clear()  # Jeda loop di dalam run()
        self.robot_pose = None

    def stop_thread(self):
        self._stop_event.set()
        self._run_event.set()  # Pastikan thread tidak terjebak di wait() saat dimatikan

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
        """Menginisialisasi node rospy hanya sekali dan memulai thread listener."""
        if not rospy: return
        try:
            # Pastikan init_node hanya dipanggil sekali
            if not rospy.core.is_initialized():
                rospy.init_node('kivy_ros_manager', anonymous=True, disable_signals=True)
            
            self.pose_listener = RosPoseListener()
            self.pose_listener.start()
            print("INFO: Node ROS 'kivy_ros_manager' berhasil diinisialisasi.")
        except rospy.ROSException:
            # Jika node sudah ada, tidak apa-apa, lanjutkan saja
            if not self.pose_listener:
                self.pose_listener = RosPoseListener()
                self.pose_listener.start()
        except Exception as e:
            print(f"FATAL: Gagal menginisialisasi node ROS: {e}")
            self.pose_listener = None

    def _stop_process_group(self, process, name):
        if process and process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired, OSError):
                pass
        return None
    
    def _send_stop_command(self):
        cancel_command = 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'
        try:
            subprocess.run(cancel_command, shell=True, check=True, timeout=2, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(0.5)
        except Exception:
            pass

        stop_command = 'rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}" -r 20'
        
        publisher_process = None
        try:
            publisher_process = subprocess.Popen(stop_command, shell=True, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(0.5)
        finally:
            if publisher_process:
                try:
                    os.killpg(os.getpgid(publisher_process.pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass

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
                
                if self.pose_listener:
                    time.sleep(5) # Beri waktu amcl untuk aktif
                    self.pose_listener.start_listening()

                print(f"INFO: Mode Navigasi dimulai dengan peta '{map_name}'.")
                return f"Navigasi dengan peta\\n'{map_name}' AKTIF"
            except Exception as e:
                print(f"FATAL: Gagal menjalankan navigasi: {e}")
                return f"GAGAL memulai navigasi!\\nError: {e}"
        return "Status: Navigasi Sudah Aktif"
        
    def stop_navigation(self):
        if self.is_navigation_running:
            if self.pose_listener:
                self.pose_listener.stop_listening()
            self._send_stop_command()
            self.navigation_process = self._stop_process_group(self.navigation_process, "Navigation")
            self.is_navigation_running = False
            self.stop_controller()
            self.current_map_name = None
            self.map_metadata = None
        return "Status: DIMATIKAN"

    def shutdown(self):
        print("INFO: Shutdown dipanggil, menghentikan semua proses...")
        if self.pose_listener:
            self.pose_listener.stop_thread()
            self.pose_listener.join()
        self._send_stop_command()
        self.stop_mapping()
        self.stop_navigation()
        self.stop_controller() 
        if self.roscore_process:
            self._stop_process_group(self.roscore_process, "roscore")
    
    def get_robot_pose(self):
        if self.pose_listener:
            return self.pose_listener.get_pose()
        return None

    # --- Sisa fungsi tidak perlu diubah dan dapat disalin dari file Anda ---
    def start_controller(self):
        if not self.is_controller_running:
            command = "roslaunch my_robot_pkg controller.launch"
            self.controller_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.is_controller_running = True
            return "Status: AKTIF"
        return "Status: Sudah Aktif"

    def stop_controller(self):
        if self.is_controller_running:
            self._send_stop_command()
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
                return "Mode Pemetaan AKTIF.\\nSilakan gerakkan robot."
            except Exception as e:
                print(f"FATAL: Gagal menjalankan mapping: {e}"); return f"GAGAL memulai mapping!\\nError: {e}"
        return "Status: Mapping Sudah Aktif"

    def stop_mapping(self):
        if self.is_mapping_running:
            self._save_map_on_exit()
            self._send_stop_command()
            self.mapping_process = self._stop_process_group(self.mapping_process, "Mapping")
            self.is_mapping_running = False
            self.stop_controller()
            self.current_map_name = None
        return "Status: DIMATIKAN"
        
    # <-- 2. FUNGSI BARU DITAMBAHKAN DI SINI
    def cancel_mapping(self):
        """Menghentikan mapping TANPA menyimpan."""
        if self.is_mapping_running:
            # self._save_map_on_exit()  <-- INILAH PERBEDAANNYA, BARIS INI DIHAPUS
            self._send_stop_command()
            self.mapping_process = self._stop_process_group(self.mapping_process, "Mapping")
            self.is_mapping_running = False
            self.stop_controller()
            self.current_map_name = None
        return "Status: DIBATALKAN"

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
        except Exception as e:
            print(f"ERROR: Gagal memuat metadata peta '{map_name}': {e}")
            self.map_metadata = None
