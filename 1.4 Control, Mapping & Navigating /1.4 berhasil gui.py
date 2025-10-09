#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from kivy.app import App
from kivy.lang import Builder
from kivy.uix.screenmanager import Screen
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.clock import mainthread, Clock
from functools import partial

from manager import RosManager

# ===== PERUBAHAN 1: Buat Class Khusus untuk Layar Pemilihan Peta =====
class NavSelectionScreen(Screen):
    def on_enter(self):
        """
        Metode ini dipanggil oleh Kivy secara otomatis SETELAH layar ini
        selesai dimuat dan ditampilkan. Ini adalah tempat paling aman.
        """
        self.update_map_list()

    def update_map_list(self):
        """Mengisi layar dengan tombol-tombol peta."""
        # 'self.ids' di sini merujuk langsung ke id di dalam layar ini, jadi lebih aman
        grid = self.ids.nav_map_grid
        grid.clear_widgets()
        
        # Dapatkan instance aplikasi utama untuk mengakses manager
        app = App.get_running_app()
        map_names = app.manager.get_available_maps()
        
        if not map_names:
            grid.add_widget(Label(text="Tidak ada peta ditemukan."))
            return
            
        for name in map_names:
            btn = Button(text=name, size_hint_y=None, height='48dp', font_size='20sp')
            # Hubungkan tombol ke fungsi di aplikasi utama
            btn.bind(on_press=partial(app.start_navigation_with_map, name))
            grid.add_widget(btn)
# =======================================================================

class MainApp(App):
    def build(self):
        self.manager = RosManager(status_callback=self.update_status_label)
        
        kv_design = """
# ===== PERUBAHAN 2: Daftarkan class baru kita di sini =====
<NavSelectionScreen>:
    BoxLayout:
        orientation: 'vertical'
        padding: 20
        spacing: 10
        Label:
            text: 'Pilih Peta untuk Navigasi'
            font_size: '24sp'
            size_hint_y: 0.15
        ScrollView:
            GridLayout:
                id: nav_map_grid # ID ini sekarang milik NavSelectionScreen
                cols: 1
                size_hint_y: None
                height: self.minimum_height
                spacing: 10
        Button:
            text: 'Kembali ke Menu'
            size_hint_y: 0.15
            on_press: root.manager.current = 'main_menu'

# ==========================================================

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
                on_press: sm.current = 'nav_selection'
                
    Screen:
        name: 'pre_mapping'
        BoxLayout:
            orientation: 'vertical'
            padding: 40
            spacing: 20
            Label:
                text: 'Masukkan Nama Peta'
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

    # ===== PERUBAHAN 3: Gunakan class baru kita di ScreenManager =====
    NavSelectionScreen:
        name: 'nav_selection'
    # ===============================================================

    Screen:
        name: 'navigation'
        BoxLayout:
            orientation: 'vertical'
            padding: 40
            spacing: 20
            Label:
                id: navigation_status_label
                text: 'Status: Siap'
                font_size: '20sp'
            Button:
                text: 'Stop Navigasi & Kembali'
                font_size: '22sp'
                on_press: app.exit_navigation_mode()
"""
        return Builder.load_string(kv_design)

    # ... (fungsi-fungsi lain tidak perlu diubah) ...
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
        if 'mapping_status_label' in screen.ids: screen.ids.mapping_status_label.text = status
        if 'current_map_name_label' in screen.ids: screen.ids.current_map_name_label.text = f"Memetakan: {map_name}"
    def exit_mapping_mode(self):
        self.update_status_label('mapping', 'mapping_status_label', 'Menyimpan peta...\nMohon tunggu.')
        Clock.schedule_once(self._finish_exit_mapping, 1)
    def _finish_exit_mapping(self, dt):
        self.manager.stop_mapping()
        self.root.current = 'main_menu'

    def start_navigation_with_map(self, map_name, *args):
        status = self.manager.start_navigation(map_name)
        self.root.current = 'navigation'
        self.update_status_label('navigation', 'navigation_status_label', status)
        
    def exit_navigation_mode(self):
        self.manager.stop_navigation()
        self.root.current = 'main_menu'
        
    def on_stop(self):
        self.manager.shutdown()

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
