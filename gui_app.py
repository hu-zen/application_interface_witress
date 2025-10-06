#!/usr/bin/env python3
import threading
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.image import Image
from kivy.graphics.texture import Texture
from kivy.clock import mainthread
import numpy as np

# Widget Kustom untuk Menampilkan Peta
class MapWidget(Image):
    @mainthread
    def update_map_texture(self, map_data, width, height):
        if self.texture is None or self.texture.size != (width, height):
            self.texture = Texture.create(size=(width, height), colorfmt='luminance')
        
        map_data_flipped = np.flipud(map_data)
        self.texture.blit_buffer(map_data_flipped.tobytes(), colorfmt='luminance', bufferfmt='ubyte')
        self.canvas.ask_update()

class GuiApp(App):
    def __init__(self, **kwargs):
        super(GuiApp, self).__init__(**kwargs)
        # Publisher untuk mengirim perintah ke robot_manager
        self.command_publisher = rospy.Publisher('/gui/robot_command', String, queue_size=10, latch=True)
        # Subscriber untuk menerima data peta dari witress_bot
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.on_map_received)
        rospy.loginfo("GUI Kivy: Siap berkomunikasi dengan witress_bot.")
        self.map_widget = None

    def build(self):
        self.title = "Witress-Bot Control Panel"
        
        # Layout utama
        main_layout = BoxLayout(orientation='vertical')
        
        # Layout untuk tombol
        button_layout = BoxLayout(size_hint_y=0.2, spacing=10, padding=10)
        
        btn_controller = Button(text="Aktifkan Kontroler")
        btn_controller.bind(on_press=self.send_command)
        btn_controller.command = "start_controller" # Tambahkan properti kustom
        
        btn_mapping = Button(text="Mulai Mapping")
        btn_mapping.bind(on_press=self.send_command)
        btn_mapping.command = "start_mapping"
        
        button_layout.add_widget(btn_controller)
        button_layout.add_widget(btn_mapping)
        
        # Widget Peta
        self.map_widget = MapWidget()
        
        main_layout.add_widget(button_layout)
        main_layout.add_widget(self.map_widget)
        return main_layout

    def send_command(self, instance):
        # Mengirim perintah yang tersimpan di tombol
        command = instance.command
        self.command_publisher.publish(command)
        print(f"GUI: Mengirim perintah '{command}' ke witress_bot.")

    def on_map_received(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.uint8)
        data[data == -1] = 127
        data[data == 0] = 255
        data[data == 100] = 0
        self.map_widget.update_map_texture(data, width, height)
        print("GUI: Peta diterima dari witress_bot dan digambar.")

    def on_stop(self):
        rospy.signal_shutdown("Aplikasi Kivy ditutup")

def run_kivy_app_in_thread():
    GuiApp().run()

if __name__ == '__main__':
    try:
        rospy.init_node('gui_node', disable_signals=True)
        rospy.loginfo("Node GUI utama telah diinisialisasi.")
        
        kivy_thread = threading.Thread(target=run_kivy_app_in_thread)
        kivy_thread.daemon = True
        kivy_thread.start()
        
        while kivy_thread.is_alive() and not rospy.is_shutdown():
            rospy.sleep(0.1)
    except KeyboardInterrupt:
        rospy.loginfo("Menutup node GUI.")
