#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ============================================================================
# == IMPORTS
# ============================================================================
import subprocess
import threading
import time
import os
import signal

# Kivy Imports
from kivy.app import App
from kivy.lang import Builder
from kivy.uix.screenmanager import Screen
from kivy.clock import mainthread

# ROS Imports
try:
    import rospy
    import rospkg
    from std_msgs.msg import String
except ImportError:
    print("\n\n[ERROR] Gagal mengimpor 'rospy'. Pastikan Anda menjalankan skrip ini")
    print("        dari terminal yang sudah menjalankan 'source ~/catkin_ws/devel/setup.bash'\n\n")
    exit()

# ============================================================================
# == KELAS ROS MANAGER (LOGIKA LATAR BELAKANG)
# ============================================================================
class RosManager:
    # ... (Isi kelas RosManager tidak ada perubahan dari versi sebelumnya) ...
    def __init__(self):
        self.controller_process = None
        self.mapping_process = None
        self.is_controller_running = False
        self.is_mapping_running = False
        self.status_callback = None
        self.save_map_publisher = rospy.Publisher('/save_map_command', String, queue_size=10)
        self.monitor_thread = threading.Thread(target=self._monitor_processes)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        print("[RosManager] Siap.")
    def set_status_callback(self, callback):
        self.status_callback = callback
    def _monitor_processes(self):
        while True:
            if self.is_controller_running and self.controller_process and self.controller_process.poll() is not None:
                self.is_controller_running = False; self.controller_process = None
                if self.status_callback: self.status_callback("controller", "controller_status_label", "Status: Gagal!")
            if self.is_mapping_running and self.mapping_process and self.mapping_process.poll() is not None:
                self.is_mapping_running = False; self.mapping_process = None
                if self.status_callback: self.status_callback("mapping", "mapping_status_label", "Status: Gagal!")
            time.sleep(1)
    def start_controller(self):
        if not self.is_controller_running:
            command = "roslaunch my_robot_pkg controller.launch"
            self.controller_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
            self.is_controller_running = True; return "Status: AKTIF"
        return "Status: Sudah Aktif"
    def stop_controller(self):
        if self.is_controller_running and self.controller_process:
            os.killpg(os.getpgid(self.controller_process.pid), signal.SIGTERM); self.controller_process.wait()
            self.controller_process = None; self.is_controller_running = False
            return "Status: DIMATIKAN"
        return "Status: Memang tidak aktif"
    def start_mapping(self):
        if not self.is_mapping_running:
            command = "roslaunch autonomus_mobile_robot mapping.launch"
            self.mapping_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
            self.is_mapping_running = True; self.start_controller()
            return "Mode Pemetaan AKTIF.\nController juga aktif."
        return "Status: Mapping Sudah Aktif"
    def stop_mapping(self):
        if self.is_mapping_running and self.mapping_process:
            os.killpg(os.getpgid(self.mapping_process.pid), signal.SIGTERM); self.mapping_process.wait()
            self.mapping_process = None; self.is_mapping_running = False
            self.stop_controller()
            return "Status: DIMATIKAN"
        return "Status: Memang tidak aktif"
    def save_map(self, map_name):
        if not map_name:
            if self.status_callback: self.status_callback("mapping", "mapping_status_label", "GAGAL: Nama peta kosong.")
            return
        print(f"INFO: Mengirim perintah (via ROS Topic) untuk menyimpan peta '{map_name}'...")
        self.save_map_publisher.publish(map_name)
        if self.status_callback: self.status_callback("mapping", "mapping_status_label", f"Perintah simpan '{map_name}'\ntelah dikirim.")
    def shutdown(self):
        print("INFO: Shutdown Manager dipanggil...")
        self.stop_mapping()

# ============================================================================
# == KELAS PROGRAM KHUSUS PENYIMPAN PETA (SAVER NODE)
# ============================================================================
class MapSaverNode:
    # ... (Isi kelas MapSaverNode tidak ada perubahan dari versi sebelumnya) ...
    def __init__(self):
        print("[MapSaverNode] Inisialisasi...")
        if not rospy.core.is_initialized():
            rospy.init_node('saver_node', anonymous=True)
        rospy.Subscriber('/save_map_command', String, self.save_map_callback)
        print("[MapSaverNode] Siap menerima perintah simpan peta.")
    def save_map_callback(self, message):
        map_name = message.data
        rospy.loginfo(f"[MapSaverNode] Menerima perintah untuk menyimpan: {map_name}")
        command_to_run = f"""
        gnome-terminal -- /bin/bash -c "source /opt/ros/noetic/setup.bash; \\
        source ~/catkin_ws/devel/setup.bash; \\
        echo 'Menyimpan peta: {map_name}'; \\
        rosrun map_server map_saver -f ~/catkin_ws/src/autonomus_mobile_robot/maps/{map_name}; \\
        echo 'Selesai. Terminal akan ditutup dalam 5 detik...'; \\
        sleep 5; exit"
        """
        try:
            subprocess.Popen(command_to_run, shell=True)
            rospy.loginfo("[MapSaverNode] Berhasil membuka terminal baru untuk map_saver.")
        except Exception as e:
            rospy.logerr(f"[MapSaverNode] Gagal membuka terminal baru: {e}")
    def spin(self):
        rospy.spin()

# ============================================================================
# == KELAS APLIKASI KIVY (TAMPILAN)
# ============================================================================
class MainApp(App):
    # ... (Isi kelas MainApp tidak ada perubahan dari versi sebelumnya) ...
    def __init__(self, ros_manager, **kwargs):
        super().__init__(**kwargs)
        self.manager = ros_manager
    def build(self):
        kv_design = """
ScreenManager:
    id: sm
    Screen:
        name: 'main_menu'
        BoxLayout:
            orientation: 'vertical'; padding: 40; spacing: 20
            Label:
                id: status_label; text: 'Waiter Bot Control Center'; font_size: '30sp'
            Button:
                text: 'Mode Controller'; font_size: '22sp'; on_press: app.go_to_controller_mode()
            Button:
                text: 'Mode Mapping'; font_size: '22sp'; on_press: app.go_to_mapping_mode()
    Screen:
        name: 'controller'
        BoxLayout:
            orientation: 'vertical'; padding: 40; spacing: 20
            Label:
                id: controller_status_label; text: 'Status: Siap'; font_size: '20sp'
            Button:
                text: 'Stop & Kembali ke Menu'; font_size: '22sp'; on_press: app.exit_controller_mode()
    Screen:
        name: 'mapping'
        BoxLayout:
            orientation: 'vertical'; padding: 40; spacing: 20
            Label:
                id: mapping_status_label; text: 'Status: Siap'; font_size: '20sp'; size_hint_y: 0.3
            TextInput:
                id: map_name_input; hint_text: 'Ketik nama peta di sini...'; font_size: '20sp'; size_hint_y: 0.2
            Button:
                text: 'Simpan Peta'; font_size: '22sp'; on_press: app.save_map()
            Button:
                text: 'Selesai Mapping & Kembali'; font_size: '22sp'; on_press: app.exit_mapping_mode()
"""
        return Builder.load_string(kv_design)
    def on_start(self):
        self.manager.set_status_callback(self.update_status_label)
    def go_to_controller_mode(self):
        status = self.manager.start_controller()
        self.update_status_label('controller', 'controller_status_label', status)
        self.root.current = 'controller'
    def exit_controller_mode(self):
        self.manager.stop_controller()
        self.root.current = 'main_menu'
    def go_to_mapping_mode(self):
        status = self.manager.start_mapping()
        self.update_status_label('mapping', 'mapping_status_label', status)
        self.root.current = 'mapping'
    def exit_mapping_mode(self):
        self.manager.stop_mapping()
        self.root.current = 'main_menu'
    def save_map(self):
        screen = self.root.get_screen('mapping')
        map_name = screen.ids.map_name_input.text
        self.manager.save_map(map_name)
    def on_stop(self):
        print("[GUI] Aplikasi ditutup, memanggil shutdown manager...")
        self.manager.shutdown()
    @mainthread
    def update_status_label(self, screen_name, label_id, new_text):
        if self.root:
            try:
                screen = self.root.get_screen(screen_name)
                if screen and label_id in screen.ids:
                    screen.ids[label_id].text = new_text
            except Exception as e:
                print(f"[GUI] Gagal update: {e}")

# ============================================================================
# == FUNGSI UTAMA (MAIN EXECUTION) - DENGAN PERBAIKAN
# ============================================================================

def wait_for_roscore(timeout=15):
    """Fungsi baru untuk menunggu roscore siap secara aktif."""
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            # Jalankan 'rostopic list'. Jika berhasil, roscore sudah siap.
            subprocess.run(['rostopic', 'list'], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("\n[Main] roscore terdeteksi aktif!")
            return True
        except (subprocess.CalledProcessError, FileNotFoundError):
            print("[Main] Menunggu roscore siap...", end='\r')
            time.sleep(1)
    print("\n[Main] Gagal mendeteksi roscore setelah {} detik.".format(timeout))
    return False

if __name__ == '__main__':
    roscore_process = None
    saver_thread = None
    try:
        # 1. Jalankan roscore
        print("[Main] Memulai roscore...")
        roscore_process = subprocess.Popen("roscore", preexec_fn=os.setsid)
        
        # 2. Tunggu roscore dengan cara yang lebih pintar, bukan dengan jeda waktu
        if not wait_for_roscore():
            raise RuntimeError("Roscore tidak dapat dimulai.")

        # 3. Setelah roscore siap, baru inisialisasi node ROS utama
        print("[Main] Inisialisasi Node ROS...")
        rospy.init_node('waiterbot_gui_main', anonymous=True)

        # 4. Jalankan "Program Khusus Penyimpan Peta" di thread terpisah
        print("[Main] Menjalankan Map Saver Node di latar belakang...")
        saver_node = MapSaverNode()
        saver_thread = threading.Thread(target=saver_node.spin)
        saver_thread.daemon = True
        saver_thread.start()

        # 5. Buat objek manager
        print("[Main] Membuat RosManager...")
        ros_manager = RosManager()
        
        # 6. Baru setelah semua siap, jalankan aplikasi Kivy
        print("[Main] Memulai Aplikasi Kivy...")
        app = MainApp(ros_manager=ros_manager)
        app.run()

    except KeyboardInterrupt:
        print("\n[Main] Ctrl+C terdeteksi, menutup aplikasi.")
    except Exception as e:
        print(f"\n[Main] Terjadi error: {e}")
    finally:
        # 7. Pastikan semua proses dimatikan saat program selesai
        print("[Main] Membersihkan semua proses...")
        if 'ros_manager' in locals() and ros_manager:
            ros_manager.shutdown()
        if roscore_process:
            print("[Main] Menghentikan roscore...")
            os.killpg(os.getpgid(roscore_process.pid), signal.SIGTERM)
            roscore_process.wait()
        print("[Main] Selesai.")
