#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.label import Label

class TestApp(App):
    def build(self):
        # Membuat layout utama
        layout = BoxLayout(orientation='vertical', padding=40, spacing=20)
        
        # Membuat label
        self.status_label = Label(text="Tekan sebuah tombol.", font_size='20sp')
        
        # Membuat tombol 1
        btn1 = Button(text='Tombol Tes A', font_size='22sp')
        btn1.bind(on_press=self.tombol_ditekan) # Menghubungkan tombol ke fungsi
        
        # Membuat tombol 2
        btn2 = Button(text='Tombol Tes B', font_size='22sp')
        btn2.bind(on_press=self.tombol_ditekan)

        # Menambahkan semua widget ke layout
        layout.add_widget(self.status_label)
        layout.add_widget(btn1)
        layout.add_widget(btn2)
        
        return layout

    def tombol_ditekan(self, instance):
        # Fungsi ini hanya akan mengubah teks label dan mencetak pesan ke terminal
        pesan = f"--- Tombol '{instance.text}' Ditekan SATU KALI ---"
        self.status_label.text = pesan
        print(pesan)

if __name__ == '__main__':
    print("==========================================================")
    print("Menjalankan Tes Kivy Murni.")
    print("Perhatikan terminal. Jika pesan 'Ditekan SATU KALI' muncul")
    print("berulang kali tanpa Anda menekan tombol, berarti ada masalah")
    print("dengan instalasi Kivy atau hardware input Anda.")
    print("==========================================================")
    TestApp().run()
