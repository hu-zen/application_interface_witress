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
from kivy.properties import ObjectProperty, NumericProperty
from kivy.uix.widget import Widget
import yaml
import math
import os

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
    # Event on_touch_down sekarang ditangani oleh parent (NavigationScreen)
    # untuk memastikan sistem koordinat yang konsisten.
    pass

# Widget untuk ikon robot, lengkap dengan instruksi rotasi di KV lang
class RobotMarker(Image):
    angle = NumericProperty(0)

class NavigationScreen(Screen):
    selected_pixel_coords = None
    robot_marker = ObjectProperty(None, allownone=True)
    click_marker = ObjectProperty(None, allownone=True)

    def on_enter(self):
        app = App.get_running_app()
        self.load_map_image(app.manager.current_map_name)
        self.ids.navigate_button.disabled = True
        
        self.cleanup_markers()

        # Buat marker robot jika belum ada, lalu sembunyikan
        if not self.robot_marker:
            icon_path = 'robot_arrow.png'
            source = icon_path if os.path.exists(icon_path) else 'atlas://data/images/defaulttheme/tree_closed'
            self.robot_marker = RobotMarker(source=source, size_hint=(None, None), size=(30, 30), allow_stretch=True)
            self.ids.map_container.add_widget(self.robot_marker)
        self.robot_marker.opacity = 0

        # Mulai timer untuk update posisi robot
        self.update_event = Clock.schedule_interval(self.update_robot_display, 0.1)

        self.selected_pixel_coords = None
        self.ids.navigation_status_label.text = "Status: Pilih titik di peta"

    def on_leave(self):
        # Hentikan timer dan bersihkan semua marker saat meninggalkan layar
        if hasattr(self, 'update_event'):
            self.update_event.cancel()
        self.cleanup_markers()

    def cleanup_markers(self):
        """Menghapus penanda X dan menyembunyikan robot."""
        if self.click_marker and self.click_marker.parent:
            self.ids.map_container.remove_widget(self.click_marker)
            self.click_marker = None
        if self.robot_marker:
            self.robot_marker.opacity = 0

    def on_map_touch(self, touch):
        """Dipanggil dari KV lang saat map_container disentuh."""
        map_viewer = self.ids.map_viewer
        # Hanya proses jika sentuhan berada di dalam area widget gambar
        if map_viewer.collide_point(*touch.pos):
            
            # Hapus penanda 'X' sebelumnya
            if self.click_marker and self.click_marker.parent:
                self.ids.map_container.remove_widget(self.click_marker)

            # Buat dan tempatkan penanda 'X' baru.
            # `touch.pos` sudah memberikan koordinat yang benar di dalam map_container.
            self.click_marker = Label(text='X', font_size='30sp', color=(1, 0, 0, 1), bold=True)
            self.click_marker.center = touch.pos
            self.ids.map_container.add_widget(self.click_marker)

            # Teruskan ke logika kalkulasi ROS
            app = App.get_running_app()
            app.calculate_ros_goal(touch, map_viewer)

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
        app = App.get_running_app()
        pose = app.manager.get_robot_pose()
        map_viewer = self.ids.map_viewer
        
        if pose is None or app.manager.map_metadata is None or not map_viewer.texture:
            if self.robot_marker: self.robot_marker.opacity = 0
            return
        
        if self.robot_marker.opacity == 0:
            self.robot_marker.opacity = 1
        
        # --- Logika Konversi Presisi Tinggi (ROS ke Kivy) ---
        meta = app.manager.map_metadata
        resolution = meta.get('resolution', 0.05)
        origin_x = meta.get('origin', [0,0,0])[0]
        origin_y = meta.get('origin', [0,0,0])[1]

        if resolution == 0: return

        # 1. Konversi posisi ROS (meter) ke koordinat piksel pada gambar asli
        pixel_x = (pose['x'] - origin_x) / resolution
        pixel_y = (pose['y'] - origin_y) / resolution

        # 2. Dapatkan ukuran tekstur (gambar) asli
        norm_w, norm_h = map_viewer.texture.size
        if norm_w == 0 or norm_h == 0: return

        # 3. Hitung skala dan offset (garis hitam) dari gambar di dalam widget
        scale_x = map_viewer.width / norm_w
        scale_y = map_viewer.height / norm_h
        scale = min(scale_x, scale_y)
        
        scaled_w = norm_w * scale
        scaled_h = norm_h * scale
        
        offset_x = (map_viewer.width - scaled_w) / 2
        offset_y = (map_viewer.height - scaled_h) / 2

        # 4. Konversi koordinat piksel peta ke posisi akhir di layar
        #    - Balik sumbu Y: `norm_h - pixel_y`
        #    - Terapkan skala: `* scale`
        #    - Tambahkan offset (garis hitam): `+ offset_`
        #    - Tambahkan posisi widget itu sendiri: `+ map_viewer.pos`
        final_x = (pixel_x * scale) + offset_x + map_viewer.x
        final_y = ((norm_h - pixel_y) * scale) + offset_y + map_viewer.y

        self.robot_marker.center = (final_x, final_y)
        self.robot_marker.angle = math.degrees(pose['yaw'])

class MainApp(App):
    def build(self):
        # Pastikan `roscore` sudah berjalan sebelum memulai aplikasi
        # Ini penting agar `rospy.init_node` di manager berhasil.
        try:
            subprocess.check_output(["pidof", "roscore"])
        except subprocess.CalledProcessError:
            print("="*50)
            print("FATAL: roscore tidak berjalan. Mohon jalankan 'roscore' di terminal lain terlebih dahulu.")
            print("="*50)
            # Opsional: langsung keluar jika roscore tidak ada
            # import sys
            # sys.exit(1)

        self.manager = RosManager(status_callback=self.update_status_label)
        
        kv_design = """
<RobotMarker>:
    canvas.before:
        PushMatrix
        Rotate:
            angle: self.angle
            origin: self.center
    canvas.after:
        PopMatrix

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
    # ... (Sisa dari ScreenManager tetap sama)
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
        screen = self.root.get_screen('navigation')
        
        if not image_widget.texture: return
        norm_w, norm_h = image_widget.texture.size
        if norm_w == 0 or norm_h == 0: return

        # Logika kalkulasi untuk ROS, sama seperti logika untuk robot
        scale_x = image_widget.width / norm_w
        scale_y = image_widget.height / norm_h
        scale = min(scale_x, scale_y)
        if scale == 0: return
        
        scaled_w = norm_w * scale
        scaled_h = norm_h * scale
        offset_x = (image_widget.width - scaled_w) / 2
        offset_y = (image_widget.height - scaled_h) / 2

        # Koordinat sentuhan relatif terhadap parent (FloatLayout)
        # Kita perlu mengubahnya menjadi relatif terhadap gambar yang diskalakan
        touch_on_image_x = touch.x - image_widget.x - offset_x
        touch_on_image_y = touch.y - image_widget.y - offset_y
        
        pixel_x_for_ros = touch_on_image_x / scale
        pixel_y_for_ros = (scaled_h - touch_on_image_y) / scale # Balik sumbu Y
        
        screen.selected_pixel_coords = (pixel_x_for_ros, pixel_y_for_ros, norm_w, norm_h)
        
        screen.ids.navigate_button.disabled = False
        screen.ids.navigation_status_label.text = "Status: Titik dipilih. Tekan 'Lakukan Navigasi'."

    def confirm_navigation_goal(self):
        screen = self.root.get_screen('navigation')
        if screen.selected_pixel_coords:
            px, py, w, h = screen.selected_pixel_coords
            self.manager.send_goal_from_pixel(px, py, w, h)
            screen.ids.navigate_button.disabled = True
            
            if screen.click_marker and screen.click_marker.parent:
                screen.ids.map_container.remove_widget(screen.click_marker)
                screen.click_marker = None

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
