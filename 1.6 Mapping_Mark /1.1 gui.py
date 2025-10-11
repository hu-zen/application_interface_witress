#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from kivy.app import App
from kivy.lang import Builder
from kivy.uix.screenmanager import Screen
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.clock import mainthread, Clock
from functools import partial
from kivy.uix.image import Image
from kivy.uix.behaviors import TouchRippleBehavior
from kivy.properties import ObjectProperty
import yaml

from manager import RosManager

class NavSelectionScreen(Screen):
    def on_enter(self):
        self.update_map_list()

    def update_map_list(self):
        grid = self.ids.nav_map_grid
        grid.clear_widgets()
        app = App.get_running_app()
        map_names = app.manager.get_available_maps()
        if not map_names:
            grid.add_widget(Label(text="Tidak ada peta ditemukan."))
            return
        for name in map_names:
            btn = Button(text=name, size_hint_y=None, height='48dp', font_size='20sp')
            btn.bind(on_press=partial(app.start_navigation_with_map, name))
            grid.add_widget(btn)

class MapImage(TouchRippleBehavior, Image):
    marker = ObjectProperty(None, allownone=True)

    def on_touch_down(self, touch):
        if self.collide_point(*touch.pos):
            if self.marker and self.marker.parent:
                self.remove_widget(self.marker)

            new_marker = Label(text='X', font_size='30sp', color=(1, 0, 0, 1), bold=True)
            new_marker.center = touch.pos
            self.add_widget(new_marker)
            self.marker = new_marker

            App.get_running_app().calculate_ros_goal(touch, self)

            return super().on_touch_down(touch)
        return False

class NavigationScreen(Screen):
    selected_pixel_coords = None

    def on_enter(self):
        app = App.get_running_app()
        self.load_map_image(app.manager.current_map_name)
        self.ids.navigate_button.disabled = True
        
        map_viewer = self.ids.map_viewer
        if map_viewer.marker and map_viewer.marker.parent:
            map_viewer.remove_widget(map_viewer.marker)
            map_viewer.marker = None

        self.selected_pixel_coords = None
        self.ids.navigation_status_label.text = "Status: Pilih titik di peta"

    def load_map_image(self, map_name):
        if map_name:
            app = App.get_running_app()
            map_image_path = app.manager.get_map_image_path(map_name)
            if map_image_path:
                self.ids.map_viewer.source = map_image_path
                app.manager.load_map_metadata(map_name)
                self.ids.map_viewer.reload()

class MainApp(App):
    def build(self):
        self.manager = RosManager(status_callback=self.update_status_label)
        
        kv_design = """
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
                id: nav_map_grid
                cols: 1
                size_hint_y: None
                height: self.minimum_height
                spacing: 10
        Button:
            text: 'Kembali ke Menu'
            size_hint_y: 0.15
            on_press: root.manager.current = 'main_menu'

<NavigationScreen>:
    name: 'navigation'
    BoxLayout:
        orientation: 'vertical'
        padding: 10
        spacing: 10
        MapImage:
            id: map_viewer
            source: ''
            allow_stretch: True
            keep_ratio: True 
            size_hint: 1, 1
        BoxLayout:
            size_hint_y: None
            height: '60dp'
            orientation: 'horizontal'
            spacing: 10
            Label:
                id: navigation_status_label
                text: 'Status: Pilih titik di peta'
                font_size: '18sp'
            Button:
                id: navigate_button
                text: 'Lakukan Navigasi'
                font_size: '20sp'
                disabled: True
                on_press: app.confirm_navigation_goal()
            Button:
                text: 'Stop & Kembali'
                font_size: '20sp'
                on_press: app.exit_navigation_mode()

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
    NavSelectionScreen:
        name: 'nav_selection'
    NavigationScreen:
        name: 'navigation'
"""
        return Builder.load_string(kv_design)

    def calculate_ros_goal(self, touch, image_widget):
        """Fungsi ini menghitung koordinat ROS dari posisi sentuhan."""
        screen = self.root.get_screen('navigation')
        
        touch_local_x, touch_local_y = touch.pos

        if not image_widget.texture: return
        widget_w, widget_h = image_widget.size
        norm_w, norm_h = image_widget.texture.size
        if norm_w == 0 or norm_h == 0: return

        widget_ratio = widget_w / widget_h if widget_h > 0 else 0
        image_ratio = norm_w / norm_h if norm_h > 0 else 0
        if image_ratio == 0: return

        if widget_ratio > image_ratio:
            scale = widget_h / norm_h
            offset_x = (widget_w - (norm_w * scale)) / 2.0
            offset_y = 0.0
        else:
            scale = widget_w / norm_w
            offset_x = 0.0
            offset_y = (widget_h - (norm_h * scale)) / 2.0
        
        if scale == 0: return

        touch_on_image_x = touch_local_x - offset_x
        touch_on_image_y = touch_local_y - offset_y

        pixel_x_for_ros = touch_on_image_x / scale
        
        # ==================================================================
        # ==================== PERBAIKAN UTAMA ADA DI SINI =================
        # ==================================================================
        # Konversi koordinat Y dari Kivy (origin di kiri-bawah) ke sistem koordinat
        # gambar (origin di kiri-atas) yang diharapkan oleh manager.py.
        pixel_y_from_bottom = touch_on_image_y / scale
        pixel_y_for_ros = norm_h - pixel_y_from_bottom # <--- INI PERUBAHANNYA
        # ==================================================================
        
        screen.selected_pixel_coords = (pixel_x_for_ros, pixel_y_for_ros, norm_w, norm_h)
        
        screen.ids.navigate_button.disabled = False
        screen.ids.navigation_status_label.text = "Status: Titik dipilih. Tekan 'Lakukan Navigasi'."

    def confirm_navigation_goal(self):
        screen = self.root.get_screen('navigation')
        if screen.selected_pixel_coords:
            px, py, w, h = screen.selected_pixel_coords
            self.manager.send_goal_from_pixel(px, py, w, h)
            screen.ids.navigate_button.disabled = True
            
            map_viewer = screen.ids.map_viewer
            if map_viewer.marker and map_viewer.marker.parent:
                map_viewer.remove_widget(map_viewer.marker)
                map_viewer.marker = None

            screen.selected_pixel_coords = None
            screen.ids.navigation_status_label.text = "Status: Perintah Goal Terkirim!"
            
    # --- Sisa fungsi tidak perlu diubah ---
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
        self.update_status_label('mapping', 'mapping_status_label', 'Menyimpan peta...\\nMohon tunggu.')
        Clock.schedule_once(self._finish_exit_mapping, 1)

    def _finish_exit_mapping(self, dt):
        self.manager.stop_mapping()
        self.root.current = 'main_menu'

    def start_navigation_with_map(self, map_name, *args):
        self.manager.start_navigation(map_name)
        self.root.current = 'navigation'

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
