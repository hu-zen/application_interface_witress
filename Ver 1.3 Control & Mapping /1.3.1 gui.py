#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from kivy.app import App
from kivy.lang import Builder
from kivy.uix.screenmanager import Screen
from kivy.clock import mainthread

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
                id: status_label # Tambahkan id ini untuk pesan error roscore
                text: 'Waiter Bot Control Center'
                font_size: '30sp'
            Button:
                text: 'Mode Controller'
                font_size: '22sp'
                on_press: app.go_to_controller_mode()
            Button:
                text: 'Mode Mapping'
                font_size: '22sp'
                on_press: app.go_to_mapping_mode()

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
                size_hint_y: 0.3
            TextInput:
                id: map_name_input
                hint_text: 'Ketik nama peta di sini...'
                font_size: '20sp'
                size_hint_y: 0.2
            Button:
                text: 'Simpan Peta'
                font_size: '22sp'
                on_press: app.save_map()
            Button:
                text: 'Selesai Mapping & Kembali'
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

    def go_to_mapping_mode(self):
        status = self.manager.start_mapping()
        self.update_status_label('mapping', 'mapping_status_label', status)
        self.root.current = 'mapping'

    def exit_mapping_mode(self):
        self.manager.stop_mapping()
        self.root.current = 'main_menu'
        
    def save_map(self):
        screen = self.root.get_screen('mapping')
        map_name = screen.ids.map_name_input.text
        
        screen.ids.mapping_status_label.text = f"Menyimpan peta '{map_name}'..."
        
        # ===== INI SATU-SATUNYA PERUBAHAN DI FILE INI =====
        self.manager.save_map_in_thread(map_name)
        # ==================================================
        
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
