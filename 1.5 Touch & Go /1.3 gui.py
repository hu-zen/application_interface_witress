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
import yaml

from manager import RosManager

# ==================================================================
# ============ AREA KALIBRASI VISUAL POSISI 'X' ==================
#
#       TIDAK DIPERLUKAN LAGI! Kita buang ini dan ganti dengan
#       perhitungan dinamis yang akurat di semua ukuran jendela.
#
# ==================================================================


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
    def on_touch_down(self, touch):
        if self.collide_point(*touch.pos):
            App.get_running_app().on_map_touch(touch, self)
            return super().on_touch_down(touch)
        return False

class NavigationScreen(Screen):
    selected_pixel_coords = None

    def on_enter(self):
        app = App.get_running_app()
        self.load_map_image(app.manager.current_map_name)
        self.ids.navigate_button.disabled = True
        self.ids.marker_layout.clear_widgets()
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
#<-- Desain KV di sini tidak berubah dari versi Anda yang berhasil -->
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
    BoxLayout:
        orientation: 'vertical'
        padding: 10
        spacing: 10
        FloatLayout:
            id: map_container
            MapImage:
                id: map_viewer
                source: ''
                allow_stretch: True
                keep_ratio: True 
                size_hint: 1, 1
                pos_hint: {'center_x': 0.5, 'center_y': 0.5}
            FloatLayout:
                id: marker_layout
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

    # ===== PERBAIKAN FINAL ADA DI SINI =====
    def on_map_touch(self, touch, image_widget):
        screen = self.root.get_screen('navigation')
        marker_layout = screen.ids.marker_layout
        
        # Pastikan gambar sudah dimuat
        if not image_widget.texture:
            return

        # 1. Dapatkan ukuran asli gambar dan ukuran widget yang menampilkannya
        image_w, image_h = image_widget.texture.size
        widget_w, widget_h = image_widget.size
        
        if image_w == 0 or image_h == 0: return

        # 2. Hitung skala dan offset secara dinamis (ini adalah kuncinya)
        scale = min(widget_w / image_w, widget_h / image_h)
        displayed_w = image_w * scale
        displayed_h = image_h * scale
        offset_x = (widget_w - displayed_w) / 2
        offset_y = (widget_h - displayed_h) / 2
        
        # 3. Ubah sentuhan (koordinat global) menjadi koordinat lokal relatif terhadap WIDGET gambar
        touch_on_widget_x = touch.x - image_widget.x
        touch_on_widget_y = touch.y - image_widget.y
        
        # 4. Pastikan sentuhan berada di dalam area gambar, bukan di area padding kosong
        if not (offset_x <= touch_on_widget_x < offset_x + displayed_w and
                offset_y <= touch_on_widget_y < offset_y + displayed_h):
            print("INFO: Sentuhan di luar area peta.")
            return
            
        # 5. Tempatkan 'X' secara visual TEPAT di posisi kursor
        marker_layout.clear_widgets()
        marker = Label(text='X', font_size='30sp', color=(1, 0, 0, 1), bold=True)
        marker.center_x = touch.x
        marker.center_y = touch.y
        marker_layout.add_widget(marker)
        
        # 6. Hitung koordinat piksel yang akurat untuk dikirim ke ROS
        touch_on_image_x = touch_on_widget_x - offset_x
        touch_on_image_y = touch_on_widget_y - offset_y
        pixel_x_for_ros = touch_on_image_x / scale
        pixel_y_for_ros = touch_on_image_y / scale
        
        # 7. Simpan hasil perhitungan & aktifkan tombol navigasi
        screen.selected_pixel_coords = (pixel_x_for_ros, pixel_y_for_ros, image_w, image_h)
        screen.ids.navigate_button.disabled = False
        screen.ids.navigation_status_label.text = "Titik dipilih. Tekan 'Lakukan Navigasi'."
    # =======================================================

    def confirm_navigation_goal(self):
        screen = self.root.get_screen('navigation')
        if screen.selected_pixel_coords:
            px, py, w, h = screen.selected_pixel_coords
            self.manager.send_goal_from_pixel(px, py, w, h)
            screen.ids.navigate_button.disabled = True
            screen.ids.marker_layout.clear_widgets()
            screen.selected_pixel_coords = None
            screen.ids.navigation_status_label.text = "Status: Perintah Goal Terkirim!"
            
    # ... Sisa fungsi tidak berubah ...
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
