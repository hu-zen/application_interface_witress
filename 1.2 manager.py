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
            # Padding: Jarak dari pinggir layar (kiri, atas, kanan, bawah)
            padding: [20, 20, 20, 20] 
            # Spacing: Jarak antar tombol
            spacing: 30 
            
            Label:
                text: 'Waiter Bot Control Center'
                font_size: '30sp'
                bold: True
                # Label kita beri tinggi tetap agar tidak memakan tempat tombol
                size_hint_y: None
                height: '50dp'

            # --- TOMBOL 1 ---
            ImageButton:
                source: 'control_robot.png'
                # [PENTING] Matikan size_hint_y agar bisa atur height manual
                size_hint_y: None
                # [ATUR DISINI] Ubah angka 150dp ini untuk memperbesar/memperkecil TINGGI gambar
                height: '150dp'
                # Opsional: Atur lebar relatif (0.8 = 80% lebar layar) agar gambar tidak gepeng
                size_hint_x: 0.8
                pos_hint: {'center_x': 0.5}
                on_press: app.go_to_controller_mode()

            # --- TOMBOL 2 ---
            ImageButton:
                source: 'make_a_map.png'
                size_hint_y: None
                # [ATUR DISINI] Sesuaikan tingginya
                height: '150dp'
                size_hint_x: 0.8
                pos_hint: {'center_x': 0.5}
                on_press: sm.current = 'pre_mapping'

            # --- TOMBOL 3 ---
            ImageButton:
                source: 'do_navigation.png'
                size_hint_y: None
                # [ATUR DISINI] Sesuaikan tingginya
                height: '150dp'
                size_hint_x: 0.8
                pos_hint: {'center_x': 0.5}
                on_press: sm.current = 'nav_selection'
                
    # ... (Screen lainnya: pre_mapping, controller, mapping, dll TETAP SAMA) ...
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
