#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from kivy.app import App
from kivy.lang import Builder
from kivy.uix.screenmanager import Screen
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
        name: 'navigation_setup'
        on_enter: app.populate_map_list()
        BoxLayout:
            orientation: 'vertical'
            padding: 40
            spacing: 20
            Label:
                text: 'Pilih Peta untuk Navigasi'
                font_size: '26sp'
            Spinner:
                id: map_spinner
                text: 'Pilih Peta...'
                font_size: '20sp'
                size_hint_y: None
                height: '48dp'
            Button:
                text: 'Mulai Navigasi'
                font_size: '22sp'
                on_press: app.start_navigation_session(map_spinner.text)
            Button:
                text: 'Kembali ke Menu'
                font_size: '22sp'
                on_press: sm.current = 'main_menu'

    Screen:
        name: 'controller'
        BoxLayout:
            orientation: 'vertical'
            padding: 40
            spacing: 20
            Label:
                id: controller_status_label
                text: 'Status: Siap'
                font_size: '20sp'
            Button:
                text: 'Stop & Kembali ke Menu'
                font_size: '22sp'
                on_press: app.exit_controller_mode()
            
    Screen:
        name: 'mapping'
        BoxLayout:
            orientation: 'vertical'
            padding: 40
            spacing: 20
            Label:
                id: mapping_status_label
                text: 'Status: Siap'
                font_size: '20sp'
            Label:
                id: current_map_name_label
                text: 'Memetakan: '
                font_size: '18sp'
                color: 0.7, 0.7, 0.7, 1
            Button:
                text: 'Selesai Mapping & Simpan Otomatis'
                font_size: '22sp'
                on_press: app.exit_mapping_mode()
                
    Screen:
        name: 'navigation_active'
        BoxLayout:
            orientation: 'vertical'
            padding: 40
            spacing: 20
            Label:
                id: navigation_status_label
                text: 'Status: Navigasi Aktif'
                font_size: '20sp'
            Label:
                text: "Gunakan '2D Nav Goal' di RViz untuk memberi perintah."
                font_size: '16sp'
            Button:
                text: 'Hentikan Navigasi & Kembali'
                font_size: '22sp'
                on_press: app.stop_navigation_session()
"""
        return Builder.load_string(kv_design)

    # --- Mode Controller ---
    def go_to_controller_mode(self):
        status = self.manager.start_controller()
        self.update_status_label('controller', 'controller_status_label', status)
        self.root.current = 'controller'

    def exit_controller_mode(self):
        self.manager.stop_controller()
        self.root.current = 'main_menu'

    # --- Mode Mapping ---
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

    # --- Mode Navigasi ---
    def go_to_navigation_setup(self):
        self.root.current = 'navigation_setup'

    def populate_map_list(self):
        map_list = self.manager.get_map_list()
        screen = self.root.get_screen('navigation_setup')
        if map_list:
            screen.ids.map_spinner.values = map_list
            screen.ids.map_spinner.text = map_list[0]
        else:
            screen.ids.map_spinner.values = []
            screen.ids.map_spinner.text = 'Tidak ada peta ditemukan'

    def start_navigation_session(self, map_name):
        if not map_name or map_name in ['Tidak ada peta ditemukan', 'Pilih Peta...']:
            # Cukup ubah teks di spinner untuk pesan error
            self.root.get_screen('navigation_setup').ids.map_spinner.text = 'Pilih peta yang valid!'
            return
            
        status = self.manager.start_navigation(map_name)
        self.root.current = 'navigation_active'
        Clock.schedule_once(lambda dt: self.update_status_label('navigation_active', 'navigation_status_label', status), 0.1)
        
    def stop_navigation_session(self):
        self.manager.stop_navigation()
        self.root.current = 'main_menu'

    # --- Fungsi Umum ---
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
