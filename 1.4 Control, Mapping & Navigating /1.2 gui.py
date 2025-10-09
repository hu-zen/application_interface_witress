#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from kivy.app import App
from kivy.lang import Builder
from kivy.uix.screenmanager import Screen
from kivy.uix.button import Button
from kivy.clock import mainthread, Clock

from manager import RosManager

class MainApp(App):
    def build(self):
        self.manager = RosManager(status_callback=self.update_status_label)
        
        kv_design = """
ScreenManager:
    id: sm
    
    Screen:
        name: 'main_menu'
        BoxLayout:
            orientation: 'vertical'
            padding: 40
            spacing: 20
            Label:
                text: 'Waiter Bot Control Center'
                font_size: '30sp'
            Button:
                text: 'Mode Controller'
                font_size: '22sp'
                on_press: app.go_to_controller_mode()
            Button:
                text: 'Mode Mapping'
                font_size: '22sp'
                on_press: sm.current = 'pre_mapping'
            Button:
                text: 'Mode Navigasi'
                font_size: '22sp'
                on_press: app.go_to_navigation_setup()

    Screen:
        name: 'pre_mapping'
        # ... (Tidak ada perubahan)
        BoxLayout:
            orientation: 'vertical'
            padding: 40
            spacing: 20
            Label:
                text: 'Masukkan Nama Peta Baru'
                font_size: '26sp'
            TextInput:
                id: map_name_input
                hint_text: 'Contoh: peta_lantai_1'
                font_size: '20sp'
                multiline: False
                size_hint_y: None
                height: '48dp'
            Button:
                text: 'Mulai Mapping'
                font_size: '22sp'
                on_press: app.go_to_mapping_mode(map_name_input.text)
            Button:
                text: 'Kembali ke Menu'
                font_size: '22sp'
                on_press: sm.current = 'main_menu'

    Screen:
        name: 'nav_selection'
        # --- PERUBAHAN DI SINI ---
        # Menghapus on_enter dari sini untuk dipanggil secara manual dari Python
        BoxLayout:
            orientation: 'vertical'
            padding: 40
            spacing: 20
            Label:
                text: 'Pilih Peta untuk Navigasi'
                font_size: '26sp'
                size_hint_y: 0.2
            ScrollView:
                GridLayout:
                    id: nav_map_grid # ID untuk GridLayout
                    cols: 1
                    size_hint_y: None
                    height: self.minimum_height
                    spacing: '10dp'
            Button:
                text: 'Kembali ke Menu'
                font_size: '22sp'
                size_hint_y: 0.2
                on_press: sm.current = 'main_menu'

    Screen:
        name: 'controller'
        # ... (Tidak ada perubahan)

    Screen:
        name: 'mapping'
        # ... (Tidak ada perubahan)
                
    Screen:
        name: 'navigation_active'
        # ... (Tidak ada perubahan)
"""
        return Builder.load_string(kv_design)

    # --- FUNGSI-FUNGSI MODE MAPPING & CONTROLLER (TIDAK BERUBAH) ---
    def go_to_controller_mode(self):
        status = self.manager.start_controller()
        self.update_status_label('controller', 'controller_status_label', status)
        self.root.current = 'controller'

    def exit_controller_mode(self):
        self.manager.stop_controller()
        self.root.current = 'main_menu'

    def go_to_mapping_mode(self, map_name):
        if not map_name.strip():
            self.root.get_screen('pre_mapping').ids.map_name_input.hint_text = 'NAMA PETA TIDAK BOLEH KOSONG!'
            return
        status = self.manager.start_mapping(map_name)
        self.root.current = 'mapping'
        Clock.schedule_once(lambda dt: self.update_mapping_labels(status, map_name), 0.1)

    def update_mapping_labels(self, status, map_name):
        screen = self.root.get_screen('mapping')
        screen.ids.mapping_status_label.text = status
        screen.ids.current_map_name_label.text = f"Memetakan: {map_name}"

    def exit_mapping_mode(self):
        self.update_status_label('mapping', 'mapping_status_label', 'Menyimpan peta dan menghentikan proses...')
        Clock.schedule_once(self._finish_exit_mapping, 1)

    def _finish_exit_mapping(self, dt):
        self.manager.stop_mapping()
        self.root.current = 'main_menu'

    # --- FUNGSI-FUNGSI MODE NAVIGASI (DIPERBAIKI) ---
    def go_to_navigation_setup(self):
        # 1. Pindah layar dulu
        self.root.current = 'nav_selection'
        # 2. Jadwalkan pemanggilan fungsi untuk mengisi daftar peta
        Clock.schedule_once(self.update_nav_map_list, 0.1)

    def update_nav_map_list(self, dt):
        """Mengambil daftar peta dan membuat tombol untuk setiap peta."""
        screen = self.root.get_screen('nav_selection')
        grid = screen.ids.nav_map_grid
        grid.clear_widgets() # Hapus tombol lama jika ada
        
        map_list = self.manager.get_map_list()

        if not map_list:
            grid.add_widget(Button(text="Tidak ada peta ditemukan. Buat peta dulu di Mode Mapping.", font_size='18sp'))
            return
            
        for map_name in map_list:
            btn = Button(text=map_name, font_size='20sp', size_hint_y=None, height='48dp')
            # Saat tombol peta ditekan, panggil start_navigation_session dengan nama peta tersebut
            btn.bind(on_press=lambda instance, name=map_name: self.start_navigation_session(name))
            grid.add_widget(btn)

    def start_navigation_session(self, map_name):
        status = self.manager.start_navigation(map_name)
        self.root.current = 'navigation_active'
        Clock.schedule_once(lambda dt: self.update_status_label('navigation_active', 'navigation_status_label', status), 0.1)
        
    def stop_navigation_session(self):
        self.manager.stop_navigation()
        self.root.current = 'main_menu'

    # --- FUNGSI UMUM (TIDAK BERUBAH) ---
    def on_stop(self):
        self.manager.shutdown()

    @mainthread
    def update_status_label(self, screen_name, label_id, new_text):
        if self.root:
            screen = self.root.get_screen(screen_name)
            if screen and label_id in screen.ids:
                screen.ids[label_id].text = new_text

if __name__ == '__main__':
    MainApp().run()
