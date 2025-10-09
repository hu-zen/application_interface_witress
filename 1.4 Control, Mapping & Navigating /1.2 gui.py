#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from kivy.app import App
from kivy.lang import Builder
from kivy.uix.screenmanager import Screen
from kivy.clock import mainthread, Clock

# Pastikan Anda mengimpor manager dari file yang benar
from manager import RosManager

class MainApp(App):
    def build(self):
        self.manager = RosManager(status_callback=self.update_status_label)
        
        # Desain KV diubah untuk mengirim teks input secara langsung
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
                id: map_name_input # ID tetap ada untuk referensi
                hint_text: 'Contoh: peta_lantai_1'
                font_size: '20sp'
                multiline: False
                size_hint_y: None
                height: '48dp'
            Button:
                text: 'Mulai Mapping'
                font_size: '22sp'
                # Kirim teks dari input langsung sebagai argumen
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
"""
        return Builder.load_string(kv_design)

    def go_to_controller_mode(self):
        status = self.manager.start_controller()
        self.update_status_label('controller', 'controller_status_label', status)
        self.root.current = 'controller'

    def exit_controller_mode(self):
        self.manager.stop_controller()
        self.root.current = 'main_menu'

    # --- FUNGSI INI YANG DIPERBAIKI ---
    def go_to_mapping_mode(self, map_name):
        if not map_name.strip():
            # Tampilkan pesan error di layar input jika nama kosong
            pre_mapping_screen = self.root.get_screen('pre_mapping')
            pre_mapping_screen.ids.map_name_input.hint_text = 'NAMA PETA TIDAK BOLEH KOSONG!'
            return

        # Mulai proses ROS di latar belakang
        status = self.manager.start_mapping(map_name)
        
        # 1. Pindah ke layar 'mapping' terlebih dahulu. Ini adalah langkah krusial.
        self.root.current = 'mapping'
        
        # 2. Jadwalkan pembaruan label. Ini akan dieksekusi setelah Kivy
        #    selesai memuat layar 'mapping', sehingga 'ids' dijamin ada.
        Clock.schedule_once(lambda dt: self.update_mapping_labels(status, map_name), 0.1)

    def update_mapping_labels(self, status, map_name):
        """Fungsi helper yang aman untuk memperbarui label di layar mapping."""
        mapping_screen = self.root.get_screen('mapping')
        # Gunakan .get() untuk menghindari KeyError jika id tidak ada karena alasan lain
        if mapping_screen.ids.get('mapping_status_label'):
            mapping_screen.ids.mapping_status_label.text = status
        if mapping_screen.ids.get('current_map_name_label'):
            mapping_screen.ids.current_map_name_label.text = f"Memetakan: {map_name}"

    def exit_mapping_mode(self):
        self.update_status_label('mapping', 'mapping_status_label', 'Menyimpan peta dan menghentikan proses...\nMohon tunggu.')
        Clock.schedule_once(self._finish_exit_mapping, 1)

    def _finish_exit_mapping(self, dt):
        self.manager.stop_mapping()
        self.root.current = 'main_menu'
        
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
