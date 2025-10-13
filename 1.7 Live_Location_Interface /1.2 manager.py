# File: manager.py

import subprocess
import time
import os
import signal
import rospkg
import glob
import yaml
import threading

# Coba impor library ROS. Jika gagal, program tetap jalan tanpa fitur real-time.
try:
    import rospy
    import tf
    from tf.transformations import euler_from_quaternion
    from kivy.clock import Clock
except ImportError:
    print("PERINGATAN: Pustaka ROS atau Kivy tidak ditemukan. Fungsionalitas penuh tidak akan bekerja.")
    rospy = None
    tf = None
    Clock = None

class RosPoseListener(threading.Thread):
    """Thread yang berjalan di latar belakang untuk mendengarkan posisi robot dari TF."""
    def __init__(self):
        super(RosPoseListener, self).__init__()
        self.daemon = True
        self.listener = None
        self.robot_pose = None
        self._stop_event = threading.Event()
        self._run_event = threading.Event()

    def run(self):
        if not rospy or not tf: return
        print("INFO (Thread): Listener posisi robot dimulai.")
        if not rospy.core.is_initialized():
            rospy.init_node('kivy_pose_listener_thread', anonymous=True, disable_signals=True)
        
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
                if not self._stop_event.is_set(): rate.sleep()
        print("INFO (Thread): Listener posisi robot dihentikan.")

    def start_listening(self): self._run_event.set()
    def stop_listening(self): self._run_event.clear(); self.robot_pose = None
    def stop_thread(self): self._stop_event.set(); self._run_event.set()
    def get_pose(self): return self.robot_pose

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

    # ==================================================================
    # ==================== LOGIKA ASINKRON BARU ========================
    # ==================================================================

    def _threaded_start_task(self, task_function, on_finish_callback, *args):
        """Helper untuk menjalankan fungsi di thread terpisah."""
        try:
            result_message = task_function(*args)
            if Clock:
                Clock.schedule_once(lambda dt: on_finish_callback(True, result_message))
        except Exception as e:
            print(f"FATAL: Gagal menjalankan proses di thread: {e}")
            if Clock:
                Clock.schedule_once(lambda dt: on_finish_callback(False, str(e)))

    def _execute_start_mapping(self, map_name):
        """Logika inti untuk memulai mapping."""
        self.current_map_name = map_name
        command = "roslaunch autonomus_mobile_robot mapping.launch"
        self.mapping_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
        self.is_mapping_running = True
        self.start_controller()
        return f"Mode Pemetaan AKTIF untuk '{map_name}'."

    def start_mapping_async(self, map_name, on_finish_callback):
        if not self.is_mapping_running:
            thread = threading.Thread(target=self._threaded_start_task, args=(self._execute_start_mapping, on_finish_callback, map_name))
            thread.daemon = True
            thread.start()
        else:
            on_finish_callback(True, "Status: Mapping Sudah Aktif")

    def _execute_start_navigation(self, map_name):
        """Logika inti untuk memulai navigasi."""
        self.current_map_name = map_name
        pkg_path = self.rospack.get_path('autonomus_mobile_robot')
        map_file_path = os.path.join(pkg_path, 'maps', f"{map_name}.yaml")
        command = f"roslaunch autonomus_mobile_robot gui_navigation.launch map_file:={map_file_path}"
        self.navigation_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
        self.is_navigation_running = True
        self.start_controller()
        if self.pose_listener:
            print("INFO: Menunggu AMCL aktif sebelum memulai listener posisi...")
            time.sleep(5) # time.sleep() sekarang aman karena ada di thread lain
            self.pose_listener.start_listening()
            print("INFO: Listener posisi diaktifkan.")
        return f"Navigasi dengan peta '{map_name}' AKTIF"

    def start_navigation_async(self, map_name, on_finish_callback):
        if not self.is_navigation_running:
            thread = threading.Thread(target=self._threaded_start_task, args=(self._execute_start_navigation, on_finish_callback, map_name))
            thread.daemon = True
            thread.start()
        else:
            on_finish_callback(True, "Status: Navigasi Sudah Aktif")

    # --- Sisa fungsi tidak perlu diubah, salin dari kode Anda sebelumnya ---
    def start_roscore_if_needed(self):
        try: subprocess.check_output(["pidof", "roscore"])
        except subprocess.CalledProcessError:
            print("INFO: roscore belum berjalan, memulai...")
            try:
                self.roscore_process = subprocess.Popen("roscore", preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(4)
            except Exception as e: print(f"FATAL: Gagal memulai roscore: {e}")
    def _init_ros_node(self):
        if not rospy: return
        try:
            if not rospy.core.is_initialized(): rospy.init_node('kivy_ros_manager', anonymous=True, disable_signals=True)
            if not self.pose_listener: self.pose_listener = RosPoseListener(); self.pose_listener.start()
            print("INFO: Node ROS 'kivy_ros_manager' berhasil diinisialisasi.")
        except (rospy.ROSException, Exception) as e: print(f"ERROR saat inisialisasi node ROS: {e}")
    def _stop_process_group(self, process, name):
        if process and process.poll() is None:
            try: os.killpg(os.getpgid(process.pid), signal.SIGTERM); process.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired, OSError): pass
        return None
    def _send_stop_command(self):
        try: subprocess.run('rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}', shell=True, check=True, timeout=2, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL); time.sleep(0.5)
        except Exception: pass
        p = None
        try:
            p = subprocess.Popen('rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}" -r 20', shell=True, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(0.5)
        finally:
            if p:
                try: os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                except ProcessLookupError: pass
    def stop_navigation(self):
        if self.is_navigation_running:
            if self.pose_listener: self.pose_listener.stop_listening()
            self._send_stop_command()
            self.navigation_process = self._stop_process_group(self.navigation_process, "Navigation")
            self.is_navigation_running = False
            self.stop_controller()
            self.current_map_name = None
            self.map_metadata = None
        return "Status: DIMATIKAN"
    def shutdown(self):
        print("INFO: Shutdown dipanggil...")
        if self.pose_listener: self.pose_listener.stop_thread(); self.pose_listener.join()
        self.stop_navigation()
        self.stop_mapping()
        self.stop_controller() 
        if self.roscore_process: self._stop_process_group(self.roscore_process, "roscore")
    def get_robot_pose(self): return self.pose_listener.get_pose() if self.pose_listener else None
    def start_controller(self):
        if not self.is_controller_running:
            self.controller_process = subprocess.Popen("roslaunch my_robot_pkg controller.launch", shell=True, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.is_controller_running = True
            return "Status: AKTIF"
        return "Status: Sudah Aktif"
    def stop_controller(self):
        if self.is_controller_running:
            self.controller_process = self._stop_process_group(self.controller_process, "Controller")
            self.is_controller_running = False
        return "Status: DIMATIKAN"
    def stop_mapping(self):
        if self.is_mapping_running:
            self._save_map_on_exit()
            self._send_stop_command()
            self.mapping_process = self._stop_process_group(self.mapping_process, "Mapping")
            self.is_mapping_running = False
            self.stop_controller()
            self.current_map_name = None
        return "Status: DIMATIKAN"
    def _save_map_on_exit(self):
        if not self.current_map_name: return
        print(f"INFO: Menyimpan peta '{self.current_map_name}'...")
        try:
            path = os.path.join(self.rospack.get_path('autonomus_mobile_robot'), 'maps', self.current_map_name)
            subprocess.run(f"rosrun map_server map_saver -f {path}", shell=True, check=True, timeout=15)
            print("INFO: Peta berhasil disimpan!")
        except Exception as e: print(f"ERROR: Gagal menyimpan peta: {e}")
    def get_available_maps(self):
        try:
            maps_dir = os.path.join(self.rospack.get_path('autonomus_mobile_robot'), 'maps')
            return [os.path.splitext(os.path.basename(f))[0] for f in glob.glob(os.path.join(maps_dir, '*.yaml'))]
        except Exception as e: print(f"ERROR: Gagal mencari peta: {e}"); return []
    def get_map_image_path(self, map_name):
        try:
            path = os.path.join(self.rospack.get_path('autonomus_mobile_robot'), 'maps', f"{map_name}.pgm")
            return path if os.path.exists(path) else None
        except Exception as e: print(f"ERROR: Gagal menemukan file gambar peta: {e}"); return None
    def load_map_metadata(self, map_name):
        try:
            path = os.path.join(self.rospack.get_path('autonomus_mobile_robot'), 'maps', f"{map_name}.yaml")
            with open(path, 'r') as f: self.map_metadata = yaml.safe_load(f)
        except Exception as e: print(f"ERROR: Gagal memuat metadata peta: {e}"); self.map_metadata = None
