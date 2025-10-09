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
        # on_enter dihapus dari sini untuk mencegah error timing
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
                    id: nav_map_grid # ID untuk GridLayout tempat tombol peta akan dibuat
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

    # --- Mode Controller (Tidak Berubah) ---
    def go_to_controller_mode(self):
        status = self.manager.start_controller()
        self.update_status_label('controller', 'controller_status_label', status)
        self.root.current = 'controller'

    def exit_controller_mode(self):
        self.manager.stop_controller()
        self.root.current = 'main_menu'

    # --- Mode Mapping (Tidak Berubah) ---
    def go_to_mapping_mode(self, map_name):
        if not map_name.strip():
            self.root.get_screen('pre_mapping').ids.map_name_input.hint_text = 'NAMA PETA TIDAK BOLEH KOSONG!'
            return
        status = self.manager.start_mapping(map_name)
        self.root.current = 'mapping'
        Clock.schedule_once(lambda dt: self.update_mapping_labels(status, map_name), 0.1)

    def update_mapping_labels(self, status, map_name):
        screen = self.root.get_screen('mapping')
        if screen.ids.get('mapping_status_label'):
            screen.ids.mapping_status_label.text = status
        if screen.ids.get('current_map_name_label'):
            screen.ids.current_map_name_label.text = f"Memetakan: {map_name}"

    def exit_mapping_mode(self):
        self.update_status_label('mapping', 'mapping_status_label', 'Menyimpan peta dan menghentikan proses...')
        Clock.schedule_once(self._finish_exit_mapping, 1)

    def _finish_exit_mapping(self, dt):
        self.manager.stop_mapping()
        self.root.current = 'main_menu'

    # --- FUNGSI-FUNGSI MODE NAVIGASI (DIPERBAIKI) ---
    def go_to_navigation_setup(self):
        # 1. Pindah layar terlebih dahulu
        self.root.current = 'nav_selection'
        # 2. Jadwalkan pemanggilan fungsi untuk mengisi daftar peta SETELAH layar dimuat
        Clock.schedule_once(self.update_nav_map_list, 0.1)

    def update_nav_map_list(self, dt):
        """Mengambil daftar peta dan membuat tombol untuk setiap peta. Ini sekarang aman."""
        try:
            screen = self.root.get_screen('nav_selection')
            grid = screen.ids.nav_map_grid # Ini sekarang tidak akan error
            grid.clear_widgets()
            
            map_list = self.manager.get_map_list()

            if not map_list:
                grid.add_widget(Button(text="Tidak ada peta. Buat peta dulu.", disabled=True, font_size='18sp'))
                return
                
            for map_name in map_list:
                btn = Button(text=map_name, font_size='20sp', size_hint_y=None, height='48dp')
                btn.bind(on_press=lambda instance, name=map_name: self.start_navigation_session(name))
                grid.add_widget(btn)
        except KeyError:
            print("ERROR: Masih terjadi KeyError. Menjadwalkan ulang...")
            # Jika karena suatu alasan masih gagal, coba lagi sekali
            Clock.schedule_once(self.update_nav_map_list, 0.2)


    def start_navigation_session(self, map_name):
        status = self.manager.start_navigation(map_name)
        self.root.current = 'navigation_active'
        Clock.schedule_once(lambda dt: self.update_status_label('navigation_active', 'navigation_status_label', status), 0.1)
        
    def stop_navigation_session(self):
        self.manager.stop_navigation()
        self.root.current = 'main_menu'

    # --- FUNGSI UMUM (Tidak Berubah) ---
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
