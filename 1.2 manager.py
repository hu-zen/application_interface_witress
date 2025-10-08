# File: manager.py (Versi Perbaikan Final)

import subprocess
import threading
import time
import os # Tambahkan ini untuk memeriksa environment

class ControllerManager:
    def __init__(self, status_callback):
        self.controller_process = None
        self.is_running = False
        self.status_callback = status_callback
        
        self.monitor_thread = threading.Thread(target=self._monitor_process)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        print("INFO: ControllerManager siap dengan pemantauan proses.")

        # Menambahkan pengecekan apakah ROS environment sudah di-source
        if 'ROS_MASTER_URI' not in os.environ:
            print("\n\nPERINGATAN PENTING:")
            print("Lingkungan ROS belum di-source. Perintah 'roslaunch' kemungkinan besar akan gagal.")
            print("Harap tutup aplikasi ini, jalankan 'source ~/catkin_ws/devel/setup.bash' di terminal, lalu jalankan kembali aplikasi ini.\n\n")


    def _monitor_process(self):
        """Fungsi ini berjalan selamanya di latar belakang."""
        while True:
            if self.is_running and self.controller_process:
                if self.controller_process.poll() is not None:
                    print("ERROR: Proses controller.launch berhenti secara tak terduga!")
                    self.is_running = False
                    self.controller_process = None
                    self.status_callback("Status: Gagal! Proses berhenti.")
            
            time.sleep(1) 

    def start_controller(self):
        """Memulai proses controller.launch."""
        if not self.is_running:
            try:
                command = "roslaunch my_robot_pkg controller.launch"
                
                # ==== PERUBAHAN UTAMA DI SINI ====
                # Kita hapus "stdout" dan "stderr" agar semua pesan dari 
                # roslaunch (termasuk roscore) muncul di terminal ini.
                self.controller_process = subprocess.Popen(command, shell=True)
                
                self.is_running = True
                print("INFO: Perintah 'start_controller' dieksekusi.")
                return "Status: AKTIF"
            except Exception as e:
                print(f"FATAL: Gagal menjalankan roslaunch: {e}")
                return "Status: Gagal total!"
        else:
            print("WARN: Controller sudah berjalan.")
            return "Status: Sudah Aktif"

    def stop_controller(self):
        """Menghentikan proses controller.launch."""
        if self.is_running and self.controller_process:
            self.controller_process.terminate()
            self.controller_process.wait()
            self.controller_process = None
            self.is_running = False
            print("INFO: Perintah 'stop_controller' dieksekusi.")
            return "Status: DIMATIKAN"
        else:
            print("INFO: Tidak ada proses untuk dihentikan.")
            return "Status: Memang tidak aktif"

    def shutdown(self):
        print("INFO: Shutdown dipanggil...")
        self.stop_controller()
