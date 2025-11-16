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
from kivy.uix.behaviors import TouchRippleBehavior, ButtonBehavior
from kivy.properties import ObjectProperty, NumericProperty
from kivy.core.window import Window
import yaml
import math
import os
import subprocess
import threading

from manager import RosManager

# ==================================
# KELAS LAYAR
# ==================================

class HomeScreen(Screen):
    pass

class NavSelectionScreen(Screen):
    def on_enter(self):
        self.update_map_list()

    def update_map_list(self):
        grid = self.ids.nav_map_grid
        grid.clear_widgets()
        app = App.get_running_app()
        map_names = app.manager.get_available_maps()
        if not map_names:
            grid.add_widget(Label(text="Tidak ada peta ditemukan.", color=(0,0,0,1)))
            return
        for name in map_names:
            # <-- 1. TOMBOL DIPERBESAR DI SINI
            btn = Button(text=name, size_hint_y=None, height='70dp', font_size='22sp')
            btn.bind(on_press=partial(app.start_navigation_with_map, name))
            grid.add_widget(btn)

class ImageButton(ButtonBehavior, Image):
    pass

class MapImage(TouchRippleBehavior, Image):
    """
    Kelas ini menggunakan logika on_touch_down dari referensi Anda.
    Tidak ada yang diubah sama sekali.
    """
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

class RobotMarker(Image):
    angle = NumericProperty(0)

class NavigationScreen(Screen):
    selected_goal_coords = None
    robot_marker = ObjectProperty(None, allownone=True)

    def on_enter(self):
        app = App.get_running_app()
        self.load_map_image(app.manager.current_map_name)
        self.ids.navigate_button.disabled = True
        
        map_viewer = self.ids.map_viewer
        if map_viewer.marker and map_viewer.marker.parent:
            map_viewer.remove_widget(map_viewer.marker)
            map_viewer.marker = None

        if not self.robot_marker:
            source = 'robot_arrow.png' if os.path.exists('robot_arrow.png') else 'atlas://data/images/defaulttheme/checkbox_on'
            self.robot_marker = RobotMarker(source=source, size_hint=(None, None), size=(30, 30), allow_stretch=True, opacity=0)
            self.ids.map_container.add_widget(self.robot_marker)
        
        self.update_event = Clock.schedule_interval(self.update_robot_display, 0.1)
        self.selected_goal_coords = None
        self.ids.navigation_status_label.text = "Status: Pilih titik di peta"

    def on_leave(self):
        if hasattr(self, 'update_event'):
            self.update_event.cancel()
        if self.robot_marker:
            self.robot_marker.opacity = 0

    def load_map_image(self, map_name):
        if map_name:
            app = App.get_running_app()
            map_image_path = app.manager.get_map_image_path(map_name)
            if map_image_path:
                self.ids.map_viewer.source = map_image_path
                app.manager.load_map_metadata(map_name)
                self.ids.map_viewer.reload()

    @mainthread
    def update_robot_display(self, dt):
        """
        Fungsi ini sekarang 100% merupakan kebalikan matematis dari
        fungsi calculate_ros_goal referensi Anda.
        """
        app = App.get_running_app()
        pose = app.manager.get_robot_pose()
        map_viewer = self.ids.map_viewer
        
        if pose is None or not app.manager.map_metadata or not map_viewer.texture:
            if self.robot_marker: self.robot_marker.opacity = 0
            return
            
        self.robot_marker.opacity = 1
        
        meta = app.manager.map_metadata
        resolution = meta.get('resolution', 0.05)
        origin_x = meta['origin'][0]
        origin_y = meta['origin'][1]

        norm_w, norm_h = map_viewer.texture.size
        if norm_w == 0 or norm_h == 0: return

        widget_w, widget_h = map_viewer.size
        img_ratio = norm_w / norm_h
        widget_ratio = widget_w / widget_h

        if widget_ratio > img_ratio:
            scale = widget_h / norm_h
            offset_x = (widget_w - norm_w * scale) / 2.0
            offset_y = 0.0
        else:
            scale = widget_w / norm_w
            offset_x = 0.0
            offset_y = (widget_h - norm_h * scale) / 2.0
        if scale == 0: return

        # === INVERSE LOGIC ===
        # 1. Konversi posisi ROS (meter) ke koordinat piksel
        pixel_x = (pose['x'] - origin_x) / resolution
        pixel_y = (pose['y'] - origin_y) / resolution

        # 2. Konversi dari piksel ke posisi di dalam widget (termasuk offset)
        pos_in_widget_x = (pixel_x * scale) + offset_x + map_viewer.x
        pos_in_widget_y = (pixel_y * scale) + offset_y + map_viewer.y
        
        self.robot_marker.center = (pos_in_widget_x, pos_in_widget_y)
        self.robot_marker.angle = -math.degrees(pose['yaw'])

# ==================================
# KELAS APLIKASI UTAMA
# ==================================
class MainApp(App):
    def build(self):
        self.manager = RosManager(status_callback=self.update_status_label)
        Window.maximize()
        
        kv_design = """
# ==================================
# ATURAN GLOBAL
# ==================================
<Screen>:
    canvas.before:
        Color:
            rgba: 1, 1, 1, 1  # Latar belakang PUTIH
        Rectangle:
            pos: self.pos
            size: self.size

<Label>:
    color: 0, 0, 0, 1  # Teks default HITAM

<Button>:
    color: 1, 1, 1, 1  # TEKS TOMBOL PUTIH
    background_color: 0.3, 0.3, 0.3, 1 # BACKGROUND ABU-ABU GELAP
    background_normal: '' 

<TextInput>:
    foreground_color: 0, 0, 0, 1 
    background_color: 0.9, 0.9, 0.9, 1 

# ==================================
# ATURAN STYLE BARU
# ==================================
<ImageButton>:
    # Membuat tombol gambar transparan
    background_color: 1, 1, 1, 0 
    background_normal: ''
    allow_stretch: True
    keep_ratio: True
    # Menambahkan efek visual "ditekan" (menjadi sedikit gelap)
    canvas.after:
        Color:
            rgba: 0, 0, 0, 0.3 if self.state == 'down' else 0
        Rectangle:
            pos: self.pos
            size: self.size

# ==================================
# ATURAN WIDGET CUSTOM
# ==================================
<RobotMarker>:
    canvas.before:
        PushMatrix
        Rotate:
            angle: self.angle
            origin: self.center
    canvas.after:
        PopMatrix
<MapImage>:
    
# ==================================
# LAYOUT LAYAR (BARU & LAMA)
# ==================================

<HomeScreen>:
    FloatLayout:
        canvas.before:
            Color:
                rgba: 1, 1, 1, 1      # PUTIH penuh
            Rectangle:
                pos: self.pos
                size: self.size
        Image:
            source: 'home.png'
            size_hint: None, None
            width: root.width * 0.58
            height: root.height * 0.58
            pos_hint: {"center_x": 0.5, "center_y": 0.65}
            allow_stretch: True
            keep_ratio: True

        # START BUTTON
        ImageButton:
            source: 'start.png'
            size_hint: None, None
            width: root.width * 0.4
            height: root.height * 0.4
            pos_hint: {"center_x": 0.5, "center_y": 0.20}
            on_press:
                app.root.current = 'main_menu'

# <-- MODIFIKASI DIMULAI DI SINI
<NavSelectionScreen>:
    BoxLayout:
        orientation: 'vertical'
        padding: 20
        spacing: 10
        
        Image:
            source: 'Choose_your_mission.png' 
            size_hint_y: 0.2
            allow_stretch: True
            keep_ratio: True
            
        # Layout horizontal untuk menampung ScrollView dan Tombol Scroll
        BoxLayout:
            orientation: 'horizontal'
            spacing: 10
            # size_hint_y: 0.6 (biarkan default agar mengisi sisa ruang)
            
            ScrollView:
                id: map_scroll # <-- ID DITAMBAHKAN
                GridLayout:
                    id: nav_map_grid
                    cols: 1
                    size_hint_y: None
                    height: self.minimum_height
                    spacing: 10
            
            # Layout vertikal untuk tombol scroll
            BoxLayout:
                orientation: 'vertical'
                size_hint_x: None
                width: '90dp' # Lebar tetap untuk tombol scroll
                spacing: 10
                
                ImageButton:
                    source: 'scroll_up.png' # <-- TOMBOL BARU
                    on_press: app.scroll_map_list_up()
                ImageButton:
                    source: 'scroll_down.png' # <-- TOMBOL BARU
                    on_press: app.scroll_map_list_down()

        ImageButton:
            source: 'go_back.png'
            size_hint_y: 0.2 
            on_press: root.manager.current = 'main_menu'
# <-- MODIFIKASI SELESAI
            
<NavigationScreen>:
    name: 'navigation'
    BoxLayout:
        orientation: 'vertical'
        padding: 10
        spacing: 10
        FloatLayout:
            id: map_container
            size_hint: 1, 1
            MapImage:
                id: map_viewer
                source: ''
                allow_stretch: True
                keep_ratio: True
                size_hint: 1, 1
                pos_hint: {'center_x': 0.5, 'center_y': 0.5}
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
            
            ImageButton:
                source: 'go_back.png'
                size_hint_y: 1.1
                on_press: app.exit_navigation_mode()

# ==================================
# SCREEN MANAGER UTAMA
# ==================================
ScreenManager:
    id: sm
    
    HomeScreen:
        name: 'home' 
        
    Screen:
        name: 'main_menu'
        BoxLayout:
            orientation: 'vertical'
            padding: 40
            spacing: 20
            Label:
                text: 'Waiter Bot Control Center'
                font_size: '30sp'
                bold: True
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
                disabled: not map_name_input.text
            ImageButton:
                source: 'go_back.png'
                size_hint_y: 0.4
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
                
            ImageButton:
                source: 'go_back.png'
                size_hint_y: 0.2
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
                color: 0.4, 0.4, 0.4, 1
                
            BoxLayout:
                orientation: 'horizontal'
                spacing: 10
                size_hint_y: None
                height: self.minimum_height
                
                Button:
                    text: 'Selesai Mapping & Simpan Otomatis'
                    font_size: '22sp'
                    size_hint_y: None 
                    height: '80dp'    
                    on_press: app.exit_mapping_mode()
                    
                Button:
                    text: 'Batalkan (Tanpa Simpan)'
                    font_size: '22sp'
                    size_hint_y: None 
                    height: '80dp'    
                    on_press: app.cancel_mapping_mode()
                    background_color: 0.8, 0.2, 0.2, 1 
            
    NavSelectionScreen:
        name: 'nav_selection'
    NavigationScreen:
        name: 'navigation'
"""
        return Builder.load_string(kv_design)

    # ==================================
    # FUNGSI-FUNGSI LOGIKA
    # ==================================

    # <-- 2. FUNGSI BARU DITAMBAHKAN DI SINI
    def scroll_map_list_up(self):
        try:
            scroll = self.root.get_screen('nav_selection').ids.map_scroll
            # Menambah scroll_y (1.0 adalah paling atas)
            new_scroll = min(1.0, scroll.scroll_y + 0.1) # 0.1 = 10%
            scroll.scroll_y = new_scroll
        except Exception as e:
            print(f"Error scrolling up: {e}")

    def scroll_map_list_down(self):
        try:
            scroll = self.root.get_screen('nav_selection').ids.map_scroll
            # Mengurangi scroll_y (0.0 adalah paling bawah)
            new_scroll = max(0.0, scroll.scroll_y - 0.1) # 0.1 = 10%
            scroll.scroll_y = new_scroll
        except Exception as e:
            print(f"Error scrolling down: {e}")
    
    # --- FUNGSI LAMA ---

    def calculate_ros_goal(self, touch, image_widget):
        """
        Fungsi ini 100% menggunakan logika dari file referensi Anda yang akurat.
        Tidak ada perubahan sama sekali.
        """
        screen = self.root.get_screen('navigation')
        
        if not image_widget.texture or not self.manager.map_metadata:
            return

        meta = self.manager.map_metadata
        resolution = meta['resolution']
        origin_x = meta['origin'][0]
        origin_y = meta['origin'][1]

        norm_w, norm_h = image_widget.texture.size
        if norm_w == 0 or norm_h == 0: return

        widget_w, widget_h = image_widget.size
        img_ratio = norm_w / norm_h
        widget_ratio = widget_w / widget_h

        if widget_ratio > img_ratio:
            scale = widget_h / norm_h
            offset_x = (widget_w - norm_w * scale) / 2.0
            offset_y = 0.0
        else:
            scale = widget_w / norm_w
            offset_x = 0.0
            offset_y = (widget_h - norm_h * scale) / 2.0
        
        if scale == 0: return

        # Kalkulasi ini menggunakan `touch.pos` (lokal) dan `offset` dari logika Anda
        # Pengurangan `image_widget.x` dan `image_widget.y` tetap dipertahankan
        # sesuai referensi akurat Anda.
        touch_on_image_x = touch.pos[0] - image_widget.x - offset_x
        touch_on_image_y = touch.pos[1] - image_widget.y - offset_y
        
        pixel_x = touch_on_image_x / scale
        pixel_y = touch_on_image_y / scale
        
        # Konversi piksel ke koordinat dunia ROS (meter)
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
                print(f"INFO: Perintah GOAL ({map_x:.2f}, {map_y:.2f}) dikirim ke /move_base_simple/goal")
            except Exception as e:
                print(f"ERROR: Gagal mengirim perintah goal: {e}")
            
            screen.ids.navigate_button.disabled = True
            screen.ids.navigation_status_label.text = "Status: Perintah Goal Terkirim!"
            
    # --- Sisa fungsi tidak diubah ---
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

    # --- Logika Threading (Sudah Benar) ---
    
    def exit_mapping_mode(self):
        self.update_status_label('mapping', 'mapping_status_label', 'Menyimpan peta...\\nMohon tunggu.')
        Clock.schedule_once(self._start_stop_mapping_thread, 0.1)

    def _start_stop_mapping_thread(self, dt):
        threading.Thread(target=self._thread_safe_stop_mapping, daemon=True).start()

    def _thread_safe_stop_mapping(self):
        self.manager.stop_mapping()
        Clock.schedule_once(self._go_to_main_menu)

    def cancel_mapping_mode(self):
        self.update_status_label('mapping', 'mapping_status_label', 'Membatalkan...\\nTidak menyimpan peta.')
        Clock.schedule_once(self._start_cancel_mapping_thread, 0.1)

    def _start_cancel_mapping_thread(self, dt):
        threading.Thread(target=self._thread_safe_cancel_mapping, daemon=True).start()

    def _thread_safe_cancel_mapping(self):
        self.manager.cancel_mapping()
        Clock.schedule_once(self._go_to_main_menu)
        
    @mainthread
    def _go_to_main_menu(self, *args):
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
