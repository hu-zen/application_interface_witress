#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import os
import signal
from kivy.app import App
from kivy.lang import Builder
from kivy.uix.screenmanager import Screen
from kivy.clock import mainthread

from manager import RosManager

class MainApp(App):
    def build(self):
        self.manager = None
        self.saver_process = None 
        
        kv_design = """
ScreenManager:
    id: sm
    
    # Layar 1: Menu Utama
    Screen:
        name: 'main_menu'
        BoxLayout:
            orientation: 'vertical'; padding: 40; spacing: 20
            Label:
                id: status_label; text: 'Waiter Bot Control Center'; font_size: '30sp'
            Button:
                text: 'Mode Controller'; font_size: '22sp'; on_press: app.go_to_controller_mode()
            Button:
                text: 'Mode Mapping'; font_size: '22sp'; on_press: root.manager.current = 'name_map'

    # Layar 2: Mode Controller (tidak berubah)
    Screen:
        name: 'controller'
        BoxLayout:
            orientation: 'vertical'; padding: 40; spacing: 20
            Label:
                id: controller_status_label; text: 'Status: Siap'; font_size: '20sp'
            Button:
                text: 'Stop & Kembali ke Menu'; font_size: '22sp'; on_press: app.exit_controller_mode()
    
    # Layar 3 (BARU): Untuk memberi nama peta
    Screen:
        name: 'name_map'
        BoxLayout:
            orientation: 'vertical'; padding: 40; spacing: 20
            Label:
                text: 'Masukkan Nama Peta Baru'
                font_size: '24sp'
                size_hint_y: 0.3
            TextInput:
                id: map_name_input
                hint_text: 'contoh: peta_dapur'
                font_size: '20sp'
                size_hint_y: 0.2
            Button:
                text: 'Lakukan Mapping'
                font_size: '22sp'
                on_press: app.start_mapping_from_input()
            Button:
                text: 'Batal'
                font_size: '22sp'
                size_hint_y: 0.2
                on_press: root.manager.current = 'main_menu'

    # Layar 4 (BARU): Saat proses mapping sedang berjalan
    Screen:
        name: 'mapping_in_progress'
        BoxLayout:
            orientation: 'vertical'; padding: 40; spacing: 20
            Label:
                id: mapping_status_label
                text: 'Status: Siap'
                font_size: '20sp'
            Button:
                text: 'Selesai & Simpan Peta'
                font_size: '22sp'
                on_press: app.finish_mapping_and_save()
            Button:
                text: 'Batalkan Mapping'
                font_size: '22sp'
                size_hint_y: 0.3
                on_press: app.cancel_mapping()
"""
        return Builder.load_string(kv_design)

    def on_start(self):
        print("INFO: [GUI] Menjalankan 'saver_node.py' di latar belakang...")
        try:
            saver_script_path = os.path.join(os.path.dirname(__file__), 'saver_node.py')
            self.saver_process = subprocess.Popen(['python3', saver_script_path])
        except Exception as e:
            print(f"FATAL: [GUI] Gagal menjalankan saver_node.py: {e}")

        self.manager = RosManager(status_callback=self.update_status_label)

    # --- Logika untuk Mode Controller (tidak berubah) ---
    def go_to_controller_mode(self):
        status = self.manager.start_controller()
        self.update_status_label('controller', 'controller_status_label', status)
        self.root.current = 'controller'
    def exit_controller_mode(self):
        self.manager.stop_controller()
        self.root.current = 'main_menu'

    # --- Logika BARU untuk alur kerja Mapping ---
    def start_mapping_from_input(self):
        """Dipanggil dari layar 'name_map'."""
        screen = self.root.get_screen('name_map')
        map_name = screen.ids.map_name_input.text
        if not map_name:
            # Bisa tambahkan popup error jika mau, untuk sekarang cukup print
            print("ERROR: Nama peta tidak boleh kosong")
            return
            
        status = self.manager.start_mapping(map_name)
        self.update_status_label('mapping_in_progress', 'mapping_status_label', status)
        self.root.current = 'mapping_in_progress'

    def finish_mapping_and_save(self):
        """Dipanggil dari layar 'mapping_in_progress'."""
        status = self.manager.finish_and_save_mapping()
        self.update_status_label('main_menu', 'status_label', "Peta telah disimpan.")
        self.root.current = 'main_menu'

    def cancel_mapping(self):
        """Dipanggil untuk membatalkan mapping dari layar 'mapping_in_progress'."""
        self.manager.cancel_mapping()
        self.root.current = 'main_menu'
        
    def on_stop(self):
        if self.manager: self.manager.shutdown()
        if self.saver_process:
            print(f"INFO: [GUI] Menghentikan 'saver_node.py' (PID: {self.saver_process.pid})")
            self.saver_process.terminate()
            self.saver_process.wait()

    @mainthread
    def update_status_label(self, screen_name, label_id, new_text):
        if self.root:
            try:
                screen = self.root.get_screen(screen_name)
                if screen and label_id in screen.ids:
                    screen.ids[label_id].text = new_text
            except Exception as e:
                print(f"Gagal update GUI: {e}")

if __name__ == '__main__':
    MainApp().run()
