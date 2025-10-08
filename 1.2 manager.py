# File: manager.py (Versi dengan roscore terpisah)

import subprocess
import threading
import time
import os
import signal

class ControllerManager:
    def __init__(self, status_callback):
        self.roscore_process = None     # <-- BARU: Proses khusus untuk roscore
        self.controller_process = None
        self.is_running = False
        self.status_callback = status_callback
        
        # Langsung jalankan roscore saat manager dibuat
        self.start_roscore()
        
        self.monitor_thread = threading.Thread(target=self._monitor_process)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        print("INFO: ControllerManager siap dengan pemantauan proses.")

    def start_roscore(self):
        """Fungsi baru untuk menjalankan roscore secara terpisah."""
        if self.roscore_process is None:
            print("INFO: Memulai roscore di latar belakang...")
            try:
                # Jalankan roscore dan sembunyikan output standarnya agar tidak ramai
                self.roscore_process = subprocess.Popen("roscore", preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(3) # Beri waktu beberapa detik agar roscore siap sepenuhnya
                print("INFO: roscore seharusnya sudah berjalan.")
            except Exception as e:
                print(f"FATAL: Gagal memulai roscore: {e}")
                # Jika roscore gagal, beri tahu GUI
                self.status_callback("Status: FATAL! Gagal memulai roscore.")
        else:
            print("INFO: roscore sudah berjalan.")

    def _monitor_process(self):
        while True:
            if self.is_running and self.controller_process:
                if self.controller_process.poll() is not None:
                    print("ERROR: Proses controller.launch berhenti tak terduga!")
                    self.is_running = False
                    self.controller_process = None
                    self.status_callback("Status: Gagal! Proses berhenti.")
            
            time.sleep(1)

    def start_controller(self):
        if not self.is_running:
            try:
                command = "roslaunch my_robot_pkg controller.launch"
                self.controller_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
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
        if self.is_running and self.controller_process:
            os.killpg(os.getpgid(self.controller_process.pid), signal.SIGTERM)
            self.controller_process.wait()
            
            self.controller_process = None
            self.is_running = False
            print("INFO: Perintah 'stop_controller' dieksekusi.")
            return "Status: DIMATIKAN"
        else:
            print("INFO: Tidak ada proses untuk dihentikan.")
            return "Status: Memang tidak aktif"

    def shutdown(self):
        """Saat aplikasi ditutup, matikan semua proses."""
        print("INFO: Shutdown dipanggil, menghentikan semua proses...")
        # Matikan controller dulu
        self.stop_controller()
        
        # BARU: Matikan roscore hanya di akhir
        if self.roscore_process:
            print("INFO: Menghentikan roscore...")
            os.killpg(os.getpgid(self.roscore_process.pid), signal.SIGTERM)
            self.roscore_process.wait()
