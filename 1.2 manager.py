# File: manager.py (Versi Perbaikan untuk Masalah Stop)

import subprocess
import threading
import time
import os       # <-- PERUBAHAN 1: Tambahkan import os
import signal   # <-- PERUBAHAN 2: Tambahkan import signal

class ControllerManager:
    def __init__(self, status_callback):
        self.controller_process = None
        self.is_running = False
        self.status_callback = status_callback
        
        self.monitor_thread = threading.Thread(target=self._monitor_process)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        print("INFO: ControllerManager siap.")

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
                
                # ==== PERUBAHAN 3: Tambahkan preexec_fn ====
                # Ini untuk memberikan ID grup proses yang unik, agar mudah dimatikan semua.
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
            # ==== PERUBAHAN 4: Gunakan metode killpg yang lebih kuat ====
            # Ini akan menghentikan seluruh grup proses (roslaunch dan semua anaknya)
            os.killpg(os.getpgid(self.controller_process.pid), signal.SIGTERM)
            self.controller_process.wait()
            
            self.controller_process = None
            self.is_running = False
            print("INFO: Perintah 'stop_controller' dieksekusi dengan killpg.")
            return "Status: DIMATIKAN"
        else:
            print("INFO: Tidak ada proses untuk dihentikan.")
            return "Status: Memang tidak aktif"

    def shutdown(self):
        print("INFO: Shutdown dipanggil...")
        self.stop_controller()
