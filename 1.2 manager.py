# File: manager.py

import subprocess

class ControllerManager:
    def __init__(self):
        """Saat kelas ini dibuat, pastikan tidak ada proses yang berjalan."""
        self.controller_process = None
        print("INFO: ControllerManager siap.")

    def start_controller(self):
        """Memulai proses controller.launch."""
        if self.controller_process is None:
            command = "roslaunch my_robot_pkg controller.launch"
            self.controller_process = subprocess.Popen(command, shell=True)
            print("INFO: Perintah 'start_controller' dieksekusi.")
            return "Status: AKTIF"
        else:
            print("WARN: Controller sudah berjalan.")
            return "Status: Sudah Aktif"

    def stop_controller(self):
        """Menghentikan proses controller.launch."""
        if self.controller_process:
            self.controller_process.terminate()
            self.controller_process.wait()
            self.controller_process = None
            print("INFO: Perintah 'stop_controller' dieksekusi.")
            return "Status: DIMATIKAN"
        else:
            print("INFO: Tidak ada proses untuk dihentikan.")
            return "Status: Memang tidak aktif"

    def shutdown(self):
        """Memastikan semua proses berhenti saat aplikasi ditutup."""
        print("INFO: Shutdown dipanggil, menghentikan semua proses...")
        self.stop_controller()
