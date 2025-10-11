# File: manager.py

import subprocess
import time
import os
import signal
import rospkg
import glob
import yaml

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
        
        self.start_roscore()
        print("INFO: RosManager siap.")

    def start_roscore(self):
        try:
            subprocess.check_output(["pidof", "roscore"])
        except subprocess.CalledProcessError:
            try:
                self.roscore_process = subprocess.Popen("roscore", preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(4)
            except Exception as e:
                print(f"FATAL: Gagal memulai roscore: {e}")

    def _stop_process_group(self, process, name):
        if process and process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired, OSError):
                # Pesan peringatan dihilangkan
                pass
        return None

    def _send_stop_command(self):
        """Menghentikan robot dengan membatalkan goal dan membanjiri /cmd_vel."""
        cancel_command = 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'
        try:
            subprocess.run(cancel_command, shell=True, check=True, timeout=2, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(0.5)
        except Exception:
            # Gagal membatalkan goal, lanjutkan dengan metode paksa (tanpa pesan)
            pass

        stop_command = 'rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}" -r 20'
        
        publisher_process = None
        try:
            publisher_process = subprocess.Popen(stop_command, shell=True, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(0.5)
        except Exception as e:
            print(f"ERROR: Gagal memulai publisher cadangan: {e}")
        finally:
            if publisher_process:
                try:
                    os.killpg(os.getpgid(publisher_process.pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass

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

    def _save_map_on_exit(self):
        if not self.current_map_name: return
        try:
            pkg_path = self.rospack.get_path('autonomus_mobile_robot')
            map_save_path = os.path.join(pkg_path, 'maps', self.current_map_name)
            command = f"rosrun map_server map_saver -f {map_save_path}"
            print(f"INFO: Menyimpan peta ke '{map_save_path}'...")
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
                
                print(f"INFO: Mode Navigasi dimulai dengan peta '{map_name}'.")
                return f"Navigasi dengan peta\\n'{map_name}' AKTIF"
            except Exception as e:
                print(f"FATAL: Gagal menjalankan navigasi: {e}")
                return f"GAGAL memulai navigasi!\\nError: {e}"
        return "Status: Navigasi Sudah Aktif"
        
    def stop_navigation(self):
        if self.is_navigation_running:
            self._send_stop_command()
            
            self.navigation_process = self._stop_process_group(self.navigation_process, "Navigation")
            self.is_navigation_running = False
            self.stop_controller()
            self.current_map_name = None
            self.map_metadata = None
        return "Status: DIMATIKAN"
    
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

    def send_goal_from_pixel(self, touch_x, touch_y, image_width, image_height):
        if not self.map_metadata:
            print("ERROR: Metadata peta belum dimuat. Tidak bisa mengirim goal.")
            return

        resolution = self.map_metadata['resolution']
        origin_x = self.map_metadata['origin'][0]
        origin_y = self.map_metadata['origin'][1]
        
        map_x = (touch_x * resolution) + origin_x
        map_y = (touch_y * resolution) + origin_y
        
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
        except Exception as e:
            print(f"ERROR: Gagal mengirim perintah goal: {e}")

    def shutdown(self):
        print("INFO: Shutdown dipanggil. Menghentikan semua proses...")
        self._send_stop_command()
        self.stop_mapping()
        self.stop_navigation()
        self.stop_controller() 
        if self.roscore_process:
            self._stop_process_group(self.roscore_process, "roscore")
