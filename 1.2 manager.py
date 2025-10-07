# File: manager.py (Versi Perbaikan)

import subprocess
import threading
import time

class ControllerManager:
    def __init__(self, status_callback):
        """
        Sekarang kita butuh 'status_callback'. 
        Ini adalah fungsi dari GUI yang akan kita panggil untuk update status.
        """
        self.controller_process = None
        self.is_running = False
        self.status_callback = status_callback
        
        # Buat dan jalankan thread untuk memantau proses
        self.monitor_thread = threading.Thread(target=self._monitor_process)
        self.monitor_thread.daemon = True # Agar thread mati saat program utama ditutup
        self.monitor_thread.start()
        print("INFO: ControllerManager siap dengan pemantauan proses.")

    def _monitor_process(self):
        """Fungsi ini berjalan selamanya di latar belakang."""
        while True:
            if self.is_running and self.controller_process:
                # poll() akan mengembalikan None jika proses masih berjalan
                if self.controller_process.poll() is not None:
                    print("ERROR: Proses controller.launch berhenti secara tak terduga!")
                    self.is_running = False
                    self.controller_process = None
                    # Panggil fungsi callback untuk update label di GUI
                    self.status_callback("Status: Gagal! Proses berhenti.")
            
            time.sleep(1) # Cek setiap 1 detik

    def start_controller(self):
        """Memulai proses controller.launch."""
        if not self.is_running:
            try:
                # GANTI 'my_robot_pkg' dengan nama paket Anda yang benar
                command = "roslaunch my_robot_pkg controller.launch"
                
                # Kita arahkan output-nya agar tidak mengganggu terminal utama
                self.controller_process = subprocess.Popen(command, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                
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
        """Memastikan semua proses berhenti saat aplikasi ditutup."""
        print("INFO: Shutdown dipanggil...")
        self.stop_controller()
