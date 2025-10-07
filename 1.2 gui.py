#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from kivy.app import App
from kivy.lang import Builder
from kivy.uix.boxlayout import BoxLayout

# Impor kelas manager dari file manager.py
from manager import ControllerManager

class ControllerApp(App):
    def build(self):
        self.manager = ControllerManager()
        kv_design = """
BoxLayout:
    orientation: 'vertical'
    padding: 40
    spacing: 20

    Label:
        id: status_label
        text: 'Status: Siap'
        font_size: '20sp'
        size_hint_y: 0.4

    Button:
        text: 'Mulai Mode Controller'
        font_size: '22sp'
        on_press: app.handle_start()

    Button:
        text: 'Stop Mode Controller'
        font_size: '22sp'
        on_press: app.handle_stop()
"""
        return Builder.load_string(kv_design)
        
    def handle_start(self):
        new_status = self.manager.start_controller()
        self.root.ids.status_label.text = new_status

    def handle_stop(self):
        new_status = self.manager.stop_controller()
        self.root.ids.status_label.text = new_status
            
    def on_stop(self):
        self.manager.shutdown()

if __name__ == '__main__':
    ControllerApp().run()
