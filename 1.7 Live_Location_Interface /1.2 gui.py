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
from kivy.properties import ObjectProperty, NumericProperty, ListProperty
import yaml
import math
import os
import subprocess

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
    pass

class RobotMarker(Image):
    angle = NumericProperty(0)

class NavigationScreen(Screen):
    selected_goal_coords = None
    robot_marker = ObjectProperty(None, allownone=True)
    click_marker = ObjectProperty(None, allownone=True)
    
    map_scale = NumericProperty(1.0)
    map_offset = ListProperty([0, 0])
    texture_size = ListProperty([0, 0])

    def on_enter(self):
        app = App.get_running_app()
        self.load_map_image(app.manager.current_map_name)
        self.ids.navigate_button.disabled = True
        self.cleanup_markers()

        if not self.robot_marker:
            source = 'robot_arrow.png' if os.path.exists('robot_arrow.png') else 'atlas://data/images/defaulttheme/checkbox_on'
            self.robot_marker = RobotMarker(source=source, size_hint=(None, None), size=(30, 30), allow_stretch=True)
            self.ids.map_container.add_widget(self.robot_marker)
        
        # ==================================================================
        # ============== PERBAIKAN LOGIKA KLIK 'X' (BAGIAN 1) ==============
        # ==================================================================
        # Buat marker 'X' sekali saja saat layar pertama kali dibuat, lalu sembunyikan.
        if not self.click_marker:
            self.click_marker = Label(text='X', font_size='30sp', color=(1, 0, 0, 1), bold=True, is_visible=False)
            self.ids.map_container.add_widget(self.click_marker)
        # ==================================================================
        
        self.update_event = Clock.schedule_interval(self.update_robot_display, 0.1)
        self.ids.navigation_status_label.text = "Status: Pilih titik di peta"

    def on_leave(self):
        if hasattr(self, 'update_event'):
            self.update_event.cancel()
        self.cleanup_markers()

    def cleanup_markers(self):
        # Sekarang cleanup hanya menyembunyikan marker, bukan menghapusnya
        if self.click_marker:
            self.click_marker.opacity = 0
        if self.robot_marker:
            self.robot_marker.opacity = 0

    def on_map_touch(self, touch):
        map_viewer = self.ids.map_viewer
        if map_viewer.collide_point(*touch.pos):
            # ==================================================================
            # ============== PERBAIKAN LOGIKA KLIK 'X' (BAGIAN 2) ==============
            # ==================================================================
            # Alih-alih membuat/menghapus, kita hanya memindahkan marker yang sudah ada.
            # Ini jauh lebih stabil dan akan selalu berfungsi.
            self.click_marker.center = touch.pos
            self.click_marker.opacity = 1 # Tampilkan marker
            # ==================================================================

            app = App.get_running_app()
            app.calculate_ros_goal(touch, self)

    def load_map_image(self, map_name):
        if map_name:
            app = App.get_running_app()
            map_image_path = app.manager.get_map_image_path(map_name)
            if map_image_path:
                self.ids.map_viewer.source = map_image_path
                app.manager.load_map_metadata(map_name)
                self.ids.map_viewer.reload()
    
    def recalculate_transform(self, *args):
        map_viewer = self.ids.map_viewer
        if not map_viewer.texture: return

        norm_w, norm_h = map_viewer.texture.size
        self.texture_size = [norm_w, norm_h]
        if norm_w == 0 or norm_h == 0: return

        scale_x = map_viewer.width / norm_w
        scale_y = map_viewer.height / norm_h
        self.map_scale = min(scale_x, scale_y)
        
        scaled_w = norm_w * self.map_scale
        scaled_h = norm_h * self.map_scale
        
        self.map_offset = [
            (map_viewer.width - scaled_w) / 2,
            (map_viewer.height - scaled_h) / 2
        ]

    @mainthread
    def update_robot_display(self, dt):
        app = App.get_running_app()
        pose = app.manager.get_robot_pose()
        map_viewer = self.ids.map_viewer
        
        if pose is None or not app.manager.map_metadata or self.map_scale == 0:
            if self.robot_marker: self.robot_marker.opacity = 0
            return
        
        self.robot_marker.opacity = 1
        
        meta = app.manager.map_metadata
        resolution = meta.get('resolution', 0.05)
        origin_x = meta.get('origin', [0,0,0])[0]
        origin_y = meta.get('origin', [0,0,0])[1]

        pixel_x = (pose['x'] - origin_x) / resolution
        # Balik sumbu Y untuk sistem koordinat Kivy
        pixel_y = self.texture_size[1] - ((pose['y'] - origin_y) / resolution)

        final_x = (pixel_x * self.map_scale) + self.map_offset[0] + map_viewer.x
        final_y = (pixel_y * self.map_scale) + self.map_offset[1] + map_viewer.y

        self.robot_marker.center = (final_x, final_y)
        self.robot_marker.angle = math.degrees(pose['yaw'])

class MainApp(App):
    def build(self):
        self.manager = RosManager(status_callback=self.update_status_label)
        
        kv_design = """
# Desain KV tidak berubah dari versi Anda yang berhasil
<RobotMarker>:
    canvas.before:
        PushMatrix
        Rotate:
            angle: self.angle
            origin: self.center
    canvas.after:
        PopMatrix

<NavSelectionScreen>:
    name: 'nav_selection'
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
        FloatLayout:
            id: map_container
            on_touch_down: root.on_map_touch(args[0])
            MapImage:
                id: map_viewer
                source: ''
                allow_stretch: True
                keep_ratio: True 
                size_hint: 1, 1
                pos_hint: {'center_x': 0.5, 'center_y': 0.5}
                on_size: root.recalculate_transform()
                on_pos: root.recalculate_transform()
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
    NavigationScreen:
"""
        return Builder.load_string(kv_design)

    def calculate_ros_goal(self, touch, screen):
        image_widget = screen.ids.map_viewer
        if not self.manager.map_metadata or screen.map_scale == 0:
            return

        meta = self.manager.map_metadata
        resolution = meta['resolution']
        origin_x = meta['origin'][0]
        origin_y = meta['origin'][1]
        
        touch_on_image_x = touch.pos[0] - image_widget.x - screen.map_offset[0]
        touch_on_image_y = touch.pos[1] - image_widget.y - screen.map_offset[1]
        
        pixel_x = touch_on_image_x / screen.map_scale
        pixel_y = screen.texture_size[1] - (touch_on_image_y / screen.map_scale)
        
        map_x = (pixel_x * resolution) + origin_x
        map_y = (pixel_y * resolution) + origin_y
        
        screen.selected_goal_coords = (map_x, map_y)
        
        screen.ids.navigate_button.disabled = False
        screen.ids.navigation_status_label.text = f"Goal: ({map_x:.2f}, {map_y:.2f})"

    def confirm_navigation_goal(self):
        screen = self.root.get_screen('navigation')
        if screen.selected_goal_coords:
            map_x, map_y = screen.selected_goal_coords
            
            goal_msg_yaml = f"""header:
  stamp: now
  frame_id: "map"
pose:
  position:
    x: {map_x}
    y: {map_y}
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"""

            command = f'rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "{goal_msg_yaml}"'
            try:
                subprocess.Popen(command, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print(f"INFO: Perintah GOAL ({map_x:.2f}, {map_y:.2f}) dikirim.")
            except Exception as e:
                print(f"ERROR: Gagal mengirim perintah goal: {e}")

            screen.ids.navigate_button.disabled = True
            screen.ids.navigation_status_label.text = "Status: Goal terkirim! Klik titik lain untuk goal baru."
            
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
