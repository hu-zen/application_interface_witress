#!/usr/bin/env python3
import subprocess
import sys
import os

def main():
    """
    Skrip jembatan untuk menjalankan perintah ROS dari luar lingkungan ROS.
    Argumen 1: Nama paket (misal: 'autonomus_mobile_robot')
    Argumen 2: Nama file launch (misal: 'mapping.launch')
    atau
    Argumen 1: 'save'
    Argumen 2: Path lengkap untuk menyimpan peta
    """
    if len(sys.argv) < 3:
        print("Error: Argumen tidak cukup. Butuh [mode] dan [argumen].")
        sys.exit(1)

    mode = sys.argv[1]
    arg = sys.argv[2]

    # Path ke setup.bash di catkin_ws Anda. Sesuaikan jika perlu.
    catkin_ws_path = os.path.expanduser('~/catkin_ws')
    setup_bash_path = os.path.join(catkin_ws_path, 'devel/setup.bash')

    if not os.path.exists(setup_bash_path):
        print(f"FATAL: File '{setup_bash_path}' tidak ditemukan!")
        sys.exit(1)
        
    command = ""
    if mode == "launch":
        # Mode untuk menjalankan roslaunch
        package = arg
        launch_file = sys.argv[3]
        command = f"source {setup_bash_path}; roslaunch {package} {launch_file}"
        print(f"Akan menjalankan: {command}")
        
    elif mode == "save":
        # Mode untuk menyimpan peta
        map_save_path = arg
        command = f"source {setup_bash_path}; rosrun map_server map_saver -f {map_save_path}; echo; echo 'Peta disimpan. Terminal akan tertutup dalam 5 detik.'; sleep 5"
        print(f"Akan menjalankan: {command}")

    else:
        print(f"Error: Mode '{mode}' tidak dikenali.")
        sys.exit(1)

    # Buka terminal baru dan jalankan perintah
    try:
        full_terminal_command = f'gnome-terminal -- bash -c "{command}"'
        subprocess.Popen(full_terminal_command, shell=True)
    except Exception as e:
        print(f"Gagal membuka terminal baru: {e}")

if __name__ == '__main__':
    main()
