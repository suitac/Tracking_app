import cv2
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, 
                           QVBoxLayout, QWidget, QFileDialog, QLabel,
                           QComboBox, QColorDialog, QHBoxLayout, QSlider,
                           QGroupBox, QRadioButton, QGridLayout, QSpinBox,
                           QCheckBox, QDoubleSpinBox, QSizePolicy)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
import sys

class TraceryPro(QMainWindow):
    def __init__(self):
        try:
            print("Initializing TraceryPro...")
            super().__init__()
            self.setWindowTitle("Tracery Pro")
            self.setGeometry(100, 100, 1600, 900)
            print("Window title and geometry set")
            
            # Add error handling for video processing
            self.is_processing = False
            self.error_count = 0
            self.max_errors = 3
            
            # Create main widget and layout
            main_widget = QWidget()
            self.setCentralWidget(main_widget)
            main_layout = QHBoxLayout(main_widget)
            
            # Left panel for video display
            left_panel = QWidget()
            left_layout = QVBoxLayout(left_panel)
            left_layout.setContentsMargins(0, 0, 0, 0)
            left_layout.setSpacing(0)
            
            # Video display
            self.video_label = QLabel("No image/video loaded")
            self.video_label.setAlignment(Qt.AlignCenter)
            self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.video_label.setMinimumSize(640, 360)  # Tamanho mínimo razoável
            left_layout.addWidget(self.video_label, stretch=100, alignment=Qt.AlignCenter)
            
            # Layout para controles abaixo do vídeo
            video_controls_widget = QWidget()
            video_controls_layout = QVBoxLayout(video_controls_widget)
            video_controls_layout.setContentsMargins(10, 10, 10, 10)
            video_controls_layout.setSpacing(5)
            
            # Status/feedback label
            self.status_label = QLabel("")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            video_controls_layout.addWidget(self.status_label)

            # Label de gravação
            self.recording_label = QLabel("")
            self.recording_label.setStyleSheet("color: red; font-size: 18px; font-weight: bold;")
            video_controls_layout.addWidget(self.recording_label)
            
            # Timeline label e slider
            self.timeline_label = QLabel("00:00 / 00:00")
            video_controls_layout.addWidget(self.timeline_label)
            self.timeline_slider = QSlider(Qt.Horizontal)
            self.timeline_slider.setRange(0, 0)
            self.timeline_slider.setEnabled(False)
            self.timeline_slider.sliderMoved.connect(self.seek_video)
            self.timeline_slider.setToolTip("Navegar para um ponto do vídeo")
            video_controls_layout.addWidget(self.timeline_slider)
            
            left_layout.addWidget(video_controls_widget, stretch=1)
            
            # Video controls
            controls_layout = QHBoxLayout()
            
            self.load_video_button = QPushButton("Load Video")
            self.load_video_button.clicked.connect(self.load_video)
            self.load_video_button.setToolTip("Carregar vídeo")
            controls_layout.addWidget(self.load_video_button)
            
            self.load_image_button = QPushButton("Load Image")
            self.load_image_button.clicked.connect(self.load_image)
            self.load_image_button.setToolTip("Carregar imagem")
            controls_layout.addWidget(self.load_image_button)
            
            self.play_pause_button = QPushButton("Play")
            self.play_pause_button.setToolTip("Play/Pause o vídeo")
            self.play_pause_button.clicked.connect(self.toggle_play_pause)
            controls_layout.addWidget(self.play_pause_button)
            
            self.save_button = QPushButton("Save Video")
            self.save_button.clicked.connect(self.save_video)
            self.save_button.setEnabled(False)
            self.save_button.setToolTip("Exportar vídeo gravado")
            controls_layout.addWidget(self.save_button)
            
            self.save_image_button = QPushButton("Save Image")
            self.save_image_button.clicked.connect(self.save_image)
            self.save_image_button.setEnabled(False)
            self.save_image_button.setToolTip("Exportar imagem processada")
            controls_layout.addWidget(self.save_image_button)
            
            # Add a reset button to the controls layout
            self.reset_button = QPushButton("Reset")
            self.reset_button.clicked.connect(self.reset_all)
            self.reset_button.setEnabled(False)
            self.reset_button.setToolTip("Resetar tudo")
            controls_layout.addWidget(self.reset_button)
            
            # Botão de gravação
            self.start_recording_button = QPushButton("Iniciar Gravação")
            self.start_recording_button.clicked.connect(self.start_recording)
            self.start_recording_button.setEnabled(False)
            self.start_recording_button.setToolTip("Iniciar gravação do vídeo processado")
            controls_layout.addWidget(self.start_recording_button)
            self.stop_recording_button = QPushButton("Parar Gravação")
            self.stop_recording_button.clicked.connect(self.stop_recording)
            self.stop_recording_button.setEnabled(False)
            self.stop_recording_button.setToolTip("Parar gravação do vídeo processado")
            controls_layout.addWidget(self.stop_recording_button)
            
            # --- NOVOS CONTROLES DE VELOCIDADE E TIMELINE ---
            # Slider de velocidade
            self.speed_label = QLabel("Speed: 1.0x")
            controls_layout.addWidget(self.speed_label)
            self.speed_slider = QSlider(Qt.Horizontal)
            self.speed_slider.setRange(25, 200)  # 0.25x a 2x
            self.speed_slider.setValue(100)      # 1x
            self.speed_slider.setFixedWidth(100)
            self.speed_slider.valueChanged.connect(self.update_speed)
            self.speed_slider.setToolTip("Ajustar velocidade de reprodução do vídeo")
            controls_layout.addWidget(self.speed_slider)

            left_layout.addLayout(controls_layout)
            
            # Right panel for effects
            right_panel = QWidget()
            right_layout = QVBoxLayout(right_panel)
            
            # Tracking method selection
            tracking_group = QGroupBox("Tracking Method")
            tracking_layout = QVBoxLayout()
            
            self.tracking_method = QComboBox()
            self.tracking_method.addItems([
                "Color Tracking",
                "Motion Detection",
                "Optical Flow"
            ])
            self.tracking_method.currentIndexChanged.connect(self.on_tracking_method_changed)
            tracking_layout.addWidget(self.tracking_method)
            
            # Number of targets
            targets_layout = QHBoxLayout()
            targets_layout.addWidget(QLabel("Max Targets:"))
            self.max_targets = QSpinBox()
            self.max_targets.setRange(1, 512)
            self.max_targets.setValue(10)
            self.max_targets.valueChanged.connect(self.update_display)
            targets_layout.addWidget(self.max_targets)
            tracking_layout.addLayout(targets_layout)
            
            # Sensitivity
            sensitivity_layout = QHBoxLayout()
            sensitivity_layout.addWidget(QLabel("Sensitivity:"))
            self.sensitivity_slider = QSlider(Qt.Horizontal)
            self.sensitivity_slider.setRange(1, 100)
            self.sensitivity_slider.setValue(50)
            self.sensitivity_slider.valueChanged.connect(self.update_display)
            self.sensitivity_slider.setToolTip("Ajustar sensibilidade do tracking de movimento")
            sensitivity_layout.addWidget(self.sensitivity_slider)
            tracking_layout.addLayout(sensitivity_layout)
            
            tracking_group.setLayout(tracking_layout)
            right_layout.addWidget(tracking_group)
            
            # Bounding box settings
            bbox_group = QGroupBox("Bounding Box Settings")
            bbox_layout = QVBoxLayout()
            
            # Show coordinates
            self.show_coords = QCheckBox("Show Coordinates")
            bbox_layout.addWidget(self.show_coords)
            
            # Box style
            style_layout = QHBoxLayout()
            style_layout.addWidget(QLabel("Box Style:"))
            self.box_style = QComboBox()
            self.box_style.addItems(["Solid", "Dashed", "Dotted", "Segmented"])
            style_layout.addWidget(self.box_style)
            bbox_layout.addLayout(style_layout)
            
            # Box thickness
            thickness_layout = QHBoxLayout()
            thickness_layout.addWidget(QLabel("Thickness:"))
            self.box_thickness = QSpinBox()
            self.box_thickness.setRange(1, 10)
            self.box_thickness.setValue(2)
            thickness_layout.addWidget(self.box_thickness)
            bbox_layout.addLayout(thickness_layout)
            
            bbox_group.setLayout(bbox_layout)
            right_layout.addWidget(bbox_group)
            
            # Center marker settings
            marker_group = QGroupBox("Center Marker Settings")
            marker_layout = QVBoxLayout()
            
            # Marker style
            marker_style_layout = QHBoxLayout()
            marker_style_layout.addWidget(QLabel("Marker Style:"))
            self.marker_style = QComboBox()
            self.marker_style.addItems(["Dot", "Plus", "Cross", "Circle"])
            marker_style_layout.addWidget(self.marker_style)
            marker_layout.addLayout(marker_style_layout)
            
            # Marker size
            marker_size_layout = QHBoxLayout()
            marker_size_layout.addWidget(QLabel("Marker Size:"))
            self.marker_size = QSpinBox()
            self.marker_size.setRange(1, 20)
            self.marker_size.setValue(5)
            marker_size_layout.addWidget(self.marker_size)
            marker_layout.addLayout(marker_size_layout)
            
            marker_group.setLayout(marker_layout)
            right_layout.addWidget(marker_group)
            
            # Connection settings
            connection_group = QGroupBox("Connection Settings")
            connection_layout = QVBoxLayout()
            
            # Connection type
            connection_type_layout = QHBoxLayout()
            connection_type_layout.addWidget(QLabel("Connection Type:"))
            self.connection_type = QComboBox()
            self.connection_type.addItems([
                "Sequential", "Star", "Full Mesh", "Minimum Spanning Tree"
            ])
            connection_type_layout.addWidget(self.connection_type)
            connection_layout.addLayout(connection_type_layout)
            
            # Line style
            line_style_layout = QHBoxLayout()
            line_style_layout.addWidget(QLabel("Line Style:"))
            self.line_style = QComboBox()
            self.line_style.addItems(["Solid", "Dashed", "Dotted", "Curved"])
            line_style_layout.addWidget(self.line_style)
            connection_layout.addLayout(line_style_layout)
            
            # Show arrows
            self.show_arrows = QCheckBox("Show Arrows")
            connection_layout.addWidget(self.show_arrows)
            
            connection_group.setLayout(connection_layout)
            right_layout.addWidget(connection_group)
            
            # Color settings group
            color_group = QGroupBox("Color Settings")
            color_layout = QVBoxLayout()
            
            # Trace color
            trace_color_layout = QHBoxLayout()
            trace_color_layout.addWidget(QLabel("Trace Color:"))
            self.trace_color_button = QPushButton("Select")
            self.trace_color_button.clicked.connect(lambda: self.select_color('trace'))
            self.trace_color = (0, 255, 0)  # Default green
            trace_color_layout.addWidget(self.trace_color_button)
            color_layout.addLayout(trace_color_layout)
            
            # Coordinate color
            coord_color_layout = QHBoxLayout()
            coord_color_layout.addWidget(QLabel("Coordinate Color:"))
            self.coord_color_button = QPushButton("Select")
            self.coord_color_button.clicked.connect(lambda: self.select_color('coord'))
            self.coord_color = (255, 255, 255)  # Default white
            coord_color_layout.addWidget(self.coord_color_button)
            color_layout.addLayout(coord_color_layout)
            
            # Bounding box color
            bbox_color_layout = QHBoxLayout()
            bbox_color_layout.addWidget(QLabel("Bounding Box Color:"))
            self.bbox_color_button = QPushButton("Select")
            self.bbox_color_button.clicked.connect(lambda: self.select_color('bbox'))
            self.bbox_color = (0, 255, 0)  # Default green
            bbox_color_layout.addWidget(self.bbox_color_button)
            color_layout.addLayout(bbox_color_layout)
            
            # Center marker color
            marker_color_layout = QHBoxLayout()
            marker_color_layout.addWidget(QLabel("Center Marker Color:"))
            self.marker_color_button = QPushButton("Select")
            self.marker_color_button.clicked.connect(lambda: self.select_color('marker'))
            self.marker_color = (0, 255, 0)  # Default green
            marker_color_layout.addWidget(self.marker_color_button)
            color_layout.addLayout(marker_color_layout)
            
            # Connection line color
            line_color_layout = QHBoxLayout()
            line_color_layout.addWidget(QLabel("Connection Line Color:"))
            self.line_color_button = QPushButton("Select")
            self.line_color_button.clicked.connect(lambda: self.select_color('line'))
            self.line_color = (0, 255, 0)  # Default green
            line_color_layout.addWidget(self.line_color_button)
            color_layout.addLayout(line_color_layout)
            
            # Arrow color
            arrow_color_layout = QHBoxLayout()
            arrow_color_layout.addWidget(QLabel("Arrow Color:"))
            self.arrow_color_button = QPushButton("Select")
            self.arrow_color_button.clicked.connect(lambda: self.select_color('arrow'))
            self.arrow_color = (0, 255, 0)  # Default green
            arrow_color_layout.addWidget(self.arrow_color_button)
            color_layout.addLayout(arrow_color_layout)
            
            color_group.setLayout(color_layout)
            right_layout.addWidget(color_group)
            
            # Color tracking settings
            tracking_color_group = QGroupBox("Color Tracking Settings")
            tracking_color_layout = QVBoxLayout()
            
            # Color selection for tracking
            tracking_color_select_layout = QHBoxLayout()
            tracking_color_select_layout.addWidget(QLabel("Color to Track:"))
            self.tracking_color_button = QPushButton("Select")
            self.tracking_color_button.clicked.connect(lambda: self.select_color('tracking'))
            tracking_color_select_layout.addWidget(self.tracking_color_button)
            tracking_color_layout.addLayout(tracking_color_select_layout)
            
            # Color range adjustment
            range_layout = QHBoxLayout()
            range_layout.addWidget(QLabel("Hue Range:"))
            self.hue_range_slider = QSlider(Qt.Horizontal)
            self.hue_range_slider.setRange(1, 50)
            self.hue_range_slider.setValue(10)
            self.hue_range_slider.valueChanged.connect(self.update_display)
            range_layout.addWidget(self.hue_range_slider)
            tracking_color_layout.addLayout(range_layout)
            
            # Saturation range
            sat_range_layout = QHBoxLayout()
            sat_range_layout.addWidget(QLabel("Saturation Range:"))
            self.sat_range_slider = QSlider(Qt.Horizontal)
            self.sat_range_slider.setRange(1, 100)
            self.sat_range_slider.setValue(50)
            self.sat_range_slider.valueChanged.connect(self.update_display)
            sat_range_layout.addWidget(self.sat_range_slider)
            tracking_color_layout.addLayout(sat_range_layout)
            
            # Value range
            val_range_layout = QHBoxLayout()
            val_range_layout.addWidget(QLabel("Value Range:"))
            self.val_range_slider = QSlider(Qt.Horizontal)
            self.val_range_slider.setRange(1, 100)
            self.val_range_slider.setValue(50)
            self.val_range_slider.valueChanged.connect(self.update_display)
            val_range_layout.addWidget(self.val_range_slider)
            tracking_color_layout.addLayout(val_range_layout)
            
            # Minimum area
            min_area_layout = QHBoxLayout()
            min_area_layout.addWidget(QLabel("Min Area:"))
            self.min_area_spin = QSpinBox()
            self.min_area_spin.setRange(1, 1000)
            self.min_area_spin.setValue(100)
            self.min_area_spin.valueChanged.connect(self.update_display)
            min_area_layout.addWidget(self.min_area_spin)
            tracking_color_layout.addLayout(min_area_layout)
            
            # Color preview
            self.color_preview = QLabel()
            self.color_preview.setFixedSize(50, 50)
            self.color_preview.setStyleSheet("background-color: black;")
            tracking_color_layout.addWidget(self.color_preview)
            
            tracking_color_group.setLayout(tracking_color_layout)
            right_layout.addWidget(tracking_color_group)
            
            # --- CONTROLES DE MOVIMENTO ---
            motion_group = QGroupBox("Motion Detection Settings")
            motion_layout = QVBoxLayout()

            # Blur size
            blur_layout = QHBoxLayout()
            blur_layout.addWidget(QLabel("Blur Size:"))
            self.blur_slider = QSlider(Qt.Horizontal)
            self.blur_slider.setRange(1, 21)
            self.blur_slider.setValue(5)
            self.blur_slider.setSingleStep(2)
            self.blur_slider.setPageStep(2)
            self.blur_slider.setTickInterval(2)
            self.blur_slider.setTickPosition(QSlider.TicksBelow)
            self.blur_slider.valueChanged.connect(self.update_display)
            self.blur_slider.setToolTip("Ajustar o desfoque para detecção de movimento")
            blur_layout.addWidget(self.blur_slider)
            motion_layout.addLayout(blur_layout)

            # Min area
            min_area_layout = QHBoxLayout()
            min_area_layout.addWidget(QLabel("Min Area:"))
            self.motion_min_area_spin = QSpinBox()
            self.motion_min_area_spin.setRange(1, 10000)
            self.motion_min_area_spin.setValue(500)
            self.motion_min_area_spin.valueChanged.connect(self.update_display)
            min_area_layout.addWidget(self.motion_min_area_spin)
            motion_layout.addLayout(min_area_layout)
            self.motion_min_area_spin.setToolTip("Área mínima para detectar movimento")

            # Show motion map
            self.show_motion_map = QCheckBox("Show Motion Map")
            self.show_motion_map.stateChanged.connect(self.update_display)
            self.show_motion_map.setToolTip("Mostrar mapa de movimento colorido")
            motion_layout.addWidget(self.show_motion_map)

            motion_group.setLayout(motion_layout)
            right_layout.addWidget(motion_group)
            
            # Add panels to main layout
            main_layout.addWidget(left_panel, stretch=2)
            main_layout.addWidget(right_panel, stretch=1)
            
            # Initialize variables
            self.cap = None
            self.timer = QTimer()
            self.timer.timeout.connect(self.update_frame)
            self.tracking_color = None
            self.processed_frames = []
            self.is_recording = False
            self.tracked_objects = []
            self.previous_frame = None
            
            # Initialize default colors
            self.reset_colors()
            
            # Tooltips para os principais controles
            self.load_video_button.setToolTip("Carregar vídeo")
            self.load_image_button.setToolTip("Carregar imagem")
            self.save_button.setToolTip("Exportar vídeo gravado")
            self.save_image_button.setToolTip("Exportar imagem processada")
            self.reset_button.setToolTip("Resetar tudo")
            self.start_recording_button.setToolTip("Iniciar gravação do vídeo processado")
            self.stop_recording_button.setToolTip("Parar gravação do vídeo processado")
            self.speed_slider.setToolTip("Ajustar velocidade de reprodução do vídeo")
            self.timeline_slider.setToolTip("Navegar para um ponto do vídeo")
            self.sensitivity_slider.setToolTip("Ajustar sensibilidade do tracking de movimento")
            self.blur_slider.setToolTip("Ajustar o desfoque para detecção de movimento")
            self.motion_min_area_spin.setToolTip("Área mínima para detectar movimento")
            self.show_motion_map.setToolTip("Mostrar mapa de movimento colorido")
            
        except Exception as e:
            print(f"Error during initialization: {str(e)}")
            self.show_error_message("Initialization Error", str(e))
        
    def reset_colors(self):
        self.trace_color = (0, 255, 0)  # Green
        self.coord_color = (255, 255, 255)  # White
        self.bbox_color = (0, 255, 0)  # Green
        self.marker_color = (0, 255, 0)  # Green
        self.line_color = (0, 255, 0)  # Green
        self.arrow_color = (0, 255, 0)  # Green
        
    def reset_all(self):
        # Reset video
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        
        # Reset colors
        self.reset_colors()
        
        # Reset tracking
        self.tracking_color = None
        self.tracked_objects = []
        self.processed_frames = []
        self.previous_frame = None
        
        # Reset UI
        self.color_preview.setStyleSheet("background-color: black;")
        self.hue_range_slider.setEnabled(False)
        self.sat_range_slider.setEnabled(False)
        self.val_range_slider.setEnabled(False)
        self.min_area_spin.setEnabled(False)
        self.video_label.clear()
        self.video_label.setText("No image/video loaded")
        
        # Reset controls
        self.play_pause_button.setEnabled(False)
        self.save_button.setEnabled(False)
        self.save_image_button.setEnabled(False)
        self.reset_button.setEnabled(False)
        self.tracking_color_button.setEnabled(False)
        self.start_recording_button.setEnabled(True)
        self.stop_recording_button.setEnabled(False)
        
        # Reset tracking method to default
        self.tracking_method.setCurrentIndex(0)
        
        # Reset all sliders and spinboxes to default values
        self.max_targets.setValue(10)
        self.sensitivity_slider.setValue(50)
        self.hue_range_slider.setValue(10)
        self.sat_range_slider.setValue(50)
        self.val_range_slider.setValue(50)
        self.min_area_spin.setValue(100)
        self.box_thickness.setValue(2)
        self.marker_size.setValue(5)
        
        # Reset checkboxes
        self.show_coords.setChecked(False)
        self.show_arrows.setChecked(False)
        
        # Reset comboboxes to default values
        self.box_style.setCurrentIndex(0)
        self.marker_style.setCurrentIndex(0)
        self.connection_type.setCurrentIndex(0)
        self.line_style.setCurrentIndex(0)
        
    def process_frame(self, frame):
        method = self.tracking_method.currentText()
        if method == "Color Tracking":
            return self.process_color_tracking(frame)
        elif method == "Motion Detection":
            return self.process_motion_detection(frame)
        elif method == "Optical Flow":
            return self.process_optical_flow(frame)
        return frame
        
    def process_color_tracking(self, frame):
        if self.tracking_color is None:
            return frame
            
        # Suavização para reduzir ruído
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        color = np.uint8([[self.tracking_color]])
        hsv_color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        h, s, v = hsv_color[0][0]
        
        # Get range values from sliders
        hue_range = self.hue_range_slider.value()
        sat_range = self.sat_range_slider.value()
        val_range = self.val_range_slider.value()
        
        # Ensure all values are within valid ranges
        h = int(h)
        s = int(s)
        v = int(v)
        
        # Ajuste dos limites
        if h < 10 or h > 170:  # Red color (wraps around in HSV)
            lower_bound1 = np.array([
                max(0, 0),
                max(0, s - sat_range),
                max(0, v - val_range)
            ], dtype=np.uint8)
            upper_bound1 = np.array([
                min(180, h + hue_range),
                min(255, s + sat_range),
                min(255, v + val_range)
            ], dtype=np.uint8)
            lower_bound2 = np.array([
                max(0, 180 - hue_range),
                max(0, s - sat_range),
                max(0, v - val_range)
            ], dtype=np.uint8)
            upper_bound2 = np.array([
                180,
                min(255, s + sat_range),
                min(255, v + val_range)
            ], dtype=np.uint8)
            mask1 = cv2.inRange(hsv, lower_bound1, upper_bound1)
            mask2 = cv2.inRange(hsv, lower_bound2, upper_bound2)
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            lower_bound = np.array([
                max(0, h - hue_range),
                max(0, s - sat_range),
                max(0, v - val_range)
            ], dtype=np.uint8)
            upper_bound = np.array([
                min(180, h + hue_range),
                min(255, s + sat_range),
                min(255, v + val_range)
            ], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # Operações morfológicas com kernel maior
        kernel = np.ones((15, 15), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Encontrar apenas contornos externos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filtrar contornos por área
        min_area = self.min_area_spin.value()
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
        
        # Ordenar e pegar só os maiores contornos
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:self.max_targets.value()]
        
        # Store tracked objects
        self.tracked_objects = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            self.tracked_objects.append({
                'x': x, 'y': y, 'w': w, 'h': h,
                'center': (x + w//2, y + h//2)
            })
        
        # Draw tracking results
        output_frame = self.draw_tracking_results(frame)
        
        # Se for imagem, garantir que as coordenadas sejam desenhadas
        if not hasattr(self, 'cap') or self.cap is None:
            for obj in self.tracked_objects:
                if self.show_coords.isChecked():
                    cv2.putText(output_frame, f"X:{obj['x']} Y:{obj['y']}", 
                                (obj['x'], obj['y'] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.coord_color, 1)
        
        return output_frame
        
    def process_motion_detection(self, frame):
        if self.previous_frame is None:
            self.previous_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            return frame
        # Blur size (sempre ímpar)
        blur_size = self.blur_slider.value()
        if blur_size % 2 == 0:
            blur_size += 1
        # Convert current frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (blur_size, blur_size), 0)
        prev_blur = cv2.GaussianBlur(self.previous_frame, (blur_size, blur_size), 0)
        # Calculate absolute difference
        frame_diff = cv2.absdiff(prev_blur, gray_blur)
        # Sensitivity (threshold)
        sensitivity = self.sensitivity_slider.value()
        _, thresh = cv2.threshold(frame_diff, sensitivity, 255, cv2.THRESH_BINARY)
        # Apply morphological operations
        kernel = np.ones((5,5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Filter contours by area
        min_area = self.motion_min_area_spin.value()
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
        # Sort contours by area and limit to max targets
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:self.max_targets.value()]
        # Store tracked objects
        self.tracked_objects = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            self.tracked_objects.append({
                'x': x, 'y': y, 'w': w, 'h': h,
                'center': (x + w//2, y + h//2)
            })
        # Update previous frame
        self.previous_frame = gray
        # Show motion map if checked
        if self.show_motion_map.isChecked():
            color_map = cv2.applyColorMap(thresh, cv2.COLORMAP_JET)
            blended = cv2.addWeighted(frame, 0.7, color_map, 0.3, 0)
            return self.draw_tracking_results(blended)
        # Draw tracking results
        return self.draw_tracking_results(frame)
        
    def process_optical_flow(self, frame):
        if self.previous_frame is None:
            self.previous_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            return frame
            
        # Convert current frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Calculate optical flow
        flow = cv2.calcOpticalFlowFarneback(
            self.previous_frame, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0
        )
        
        # Calculate magnitude and angle
        magnitude, angle = cv2.cartToPolar(flow[..., 0], flow[..., 1])
        
        # Threshold magnitude to find significant motion
        mask = cv2.threshold(magnitude, 2.0, 255, cv2.THRESH_BINARY)[1]
        
        # Apply morphological operations
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by area
        min_area = 500
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
        
        # Sort contours by area and limit to max targets
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:self.max_targets.value()]
        
        # Store tracked objects
        self.tracked_objects = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            self.tracked_objects.append({
                'x': x, 'y': y, 'w': w, 'h': h,
                'center': (x + w//2, y + h//2)
            })
        
        # Update previous frame
        self.previous_frame = gray
        
        # Draw tracking results
        return self.draw_tracking_results(frame)
        
    def draw_tracking_results(self, frame):
        # Draw bounding boxes
        for obj in self.tracked_objects:
            # Draw box
            if self.box_style.currentText() == "Solid":
                cv2.rectangle(frame, (obj['x'], obj['y']), 
                            (obj['x'] + obj['w'], obj['y'] + obj['h']), 
                            self.bbox_color, self.box_thickness.value())
            elif self.box_style.currentText() == "Dashed":
                self.draw_dashed_rect(frame, obj['x'], obj['y'], 
                                    obj['w'], obj['h'], self.bbox_color, 
                                    self.box_thickness.value())
            elif self.box_style.currentText() == "Dotted":
                self.draw_dotted_rect(frame, obj['x'], obj['y'], 
                                    obj['w'], obj['h'], self.bbox_color, 
                                    self.box_thickness.value())
            elif self.box_style.currentText() == "Segmented":
                self.draw_segmented_rect(frame, obj['x'], obj['y'], 
                                       obj['w'], obj['h'], self.bbox_color, 
                                       self.box_thickness.value())
            
            # Draw coordinates
            if self.show_coords.isChecked():
                cv2.putText(frame, f"X:{obj['x']} Y:{obj['y']}", 
                           (obj['x'], obj['y'] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.coord_color, 1)
            
            # Draw center marker
            self.draw_center_marker(frame, obj['center'])
        
        # Draw connections
        self.draw_connections(frame)
        
        return frame
        
    def draw_center_marker(self, frame, center):
        style = self.marker_style.currentText()
        size = self.marker_size.value()
        
        if style == "Dot":
            cv2.circle(frame, center, size, self.marker_color, -1)
        elif style == "Plus":
            cv2.line(frame, (center[0]-size, center[1]), 
                    (center[0]+size, center[1]), self.marker_color, 2)
            cv2.line(frame, (center[0], center[1]-size), 
                    (center[0], center[1]+size), self.marker_color, 2)
        elif style == "Cross":
            cv2.line(frame, (center[0]-size, center[1]-size), 
                    (center[0]+size, center[1]+size), self.marker_color, 2)
            cv2.line(frame, (center[0]-size, center[1]+size), 
                    (center[0]+size, center[1]-size), self.marker_color, 2)
        elif style == "Circle":
            cv2.circle(frame, center, size, self.marker_color, 2)
            
    def draw_connections(self, frame):
        if not self.tracked_objects:
            return
            
        connection_type = self.connection_type.currentText()
        
        if connection_type == "Sequential":
            for i in range(len(self.tracked_objects)-1):
                self.draw_connection_line(frame, 
                                       self.tracked_objects[i]['center'],
                                       self.tracked_objects[i+1]['center'])
                
        elif connection_type == "Star":
            center = self.tracked_objects[0]['center']
            for obj in self.tracked_objects[1:]:
                self.draw_connection_line(frame, center, obj['center'])
                
        elif connection_type == "Full Mesh":
            for i in range(len(self.tracked_objects)):
                for j in range(i+1, len(self.tracked_objects)):
                    self.draw_connection_line(frame,
                                           self.tracked_objects[i]['center'],
                                           self.tracked_objects[j]['center'])
                    
        elif connection_type == "Minimum Spanning Tree":
            # Create a graph of all points
            points = [obj['center'] for obj in self.tracked_objects]
            edges = []
            for i in range(len(points)):
                for j in range(i+1, len(points)):
                    dist = np.sqrt((points[i][0] - points[j][0])**2 + 
                                 (points[i][1] - points[j][1])**2)
                    edges.append((i, j, dist))
            
            # Sort edges by distance
            edges.sort(key=lambda x: x[2])
            
            # Kruskal's algorithm for MST
            parent = list(range(len(points)))
            
            def find(x):
                if parent[x] != x:
                    parent[x] = find(parent[x])
                return parent[x]
            
            # Add edges to MST
            for i, j, _ in edges:
                if find(i) != find(j):
                    parent[find(i)] = find(j)
                    self.draw_connection_line(frame, points[i], points[j])
                    
    def draw_connection_line(self, frame, start, end):
        style = self.line_style.currentText()
        
        if style == "Solid":
            cv2.line(frame, start, end, self.line_color, 2)
        elif style == "Dashed":
            self.draw_dashed_line(frame, start, end, self.line_color, 2)
        elif style == "Dotted":
            self.draw_dotted_line(frame, start, end, self.line_color, 2)
        elif style == "Curved":
            self.draw_curved_line(frame, start, end, self.line_color, 2)
            
        if self.show_arrows.isChecked():
            self.draw_arrow(frame, start, end, self.arrow_color, 2)
            
    def draw_dashed_line(self, frame, start, end, color, thickness):
        dash_length = 10
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dist = np.sqrt(dx*dx + dy*dy)
        steps = int(dist / dash_length)
        
        for i in range(steps):
            if i % 2 == 0:
                p1 = (int(start[0] + dx*i/steps), int(start[1] + dy*i/steps))
                p2 = (int(start[0] + dx*(i+1)/steps), int(start[1] + dy*(i+1)/steps))
                cv2.line(frame, p1, p2, color, thickness)
                
    def draw_dotted_line(self, frame, start, end, color, thickness):
        dot_spacing = 10
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dist = np.sqrt(dx*dx + dy*dy)
        steps = int(dist / dot_spacing)
        
        for i in range(steps):
            p = (int(start[0] + dx*i/steps), int(start[1] + dy*i/steps))
            cv2.circle(frame, p, thickness//2, color, -1)
            
    def draw_curved_line(self, frame, start, end, color, thickness):
        # Create a curved line using a quadratic Bezier curve
        control = ((start[0] + end[0])//2, start[1] - 50)  # Control point above the line
        
        steps = 50
        for i in range(steps):
            t = i / steps
            # Quadratic Bezier formula
            x = int((1-t)**2 * start[0] + 2*(1-t)*t * control[0] + t**2 * end[0])
            y = int((1-t)**2 * start[1] + 2*(1-t)*t * control[1] + t**2 * end[1])
            cv2.circle(frame, (x, y), thickness//2, color, -1)
            
    def draw_arrow(self, frame, start, end, color, thickness):
        # Calculate arrow parameters
        arrow_length = 20
        arrow_angle = np.pi/6  # 30 degrees
        
        # Calculate direction vector
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = np.sqrt(dx*dx + dy*dy)
        
        if length == 0:
            return
            
        # Normalize direction vector
        dx /= length
        dy /= length
        
        # Calculate arrow points
        angle = np.arctan2(dy, dx)
        arrow1 = (int(end[0] - arrow_length * np.cos(angle + arrow_angle)),
                 int(end[1] - arrow_length * np.sin(angle + arrow_angle)))
        arrow2 = (int(end[0] - arrow_length * np.cos(angle - arrow_angle)),
                 int(end[1] - arrow_length * np.sin(angle - arrow_angle)))
        
        # Draw arrow
        cv2.line(frame, end, arrow1, color, thickness)
        cv2.line(frame, end, arrow2, color, thickness)
        
    def draw_dashed_rect(self, frame, x, y, w, h, color, thickness):
        dash_length = 10
        for i in range(0, w, dash_length*2):
            cv2.line(frame, (x+i, y), (x+i+dash_length, y), color, thickness)
            cv2.line(frame, (x+i, y+h), (x+i+dash_length, y+h), color, thickness)
        for i in range(0, h, dash_length*2):
            cv2.line(frame, (x, y+i), (x, y+i+dash_length), color, thickness)
            cv2.line(frame, (x+w, y+i), (x+w, y+i+dash_length), color, thickness)
            
    def draw_dotted_rect(self, frame, x, y, w, h, color, thickness):
        dot_spacing = 10
        for i in range(0, w, dot_spacing):
            cv2.circle(frame, (x+i, y), thickness//2, color, -1)
            cv2.circle(frame, (x+i, y+h), thickness//2, color, -1)
        for i in range(0, h, dot_spacing):
            cv2.circle(frame, (x, y+i), thickness//2, color, -1)
            cv2.circle(frame, (x+w, y+i), thickness//2, color, -1)
            
    def draw_segmented_rect(self, frame, x, y, w, h, color, thickness):
        segment_length = 20
        gap_length = 10
        
        # Draw horizontal segments
        for i in range(0, w, segment_length + gap_length):
            end_x = min(x + i + segment_length, x + w)
            cv2.line(frame, (x+i, y), (end_x, y), color, thickness)
            cv2.line(frame, (x+i, y+h), (end_x, y+h), color, thickness)
            
        # Draw vertical segments
        for i in range(0, h, segment_length + gap_length):
            end_y = min(y + i + segment_length, y + h)
            cv2.line(frame, (x, y+i), (x, end_y), color, thickness)
            cv2.line(frame, (x+w, y+i), (x+w, end_y), color, thickness)
            
    def load_video(self):
        file_name, _ = QFileDialog.getOpenFileName(
            self, 
            "Open Video File", 
            "", 
            "Video Files (*.mp4 *.avi *.mov)"
        )
        if file_name:
            self.cap = cv2.VideoCapture(file_name)
            self.video_path = file_name  # Salva o caminho do vídeo original
            if self.cap.isOpened():
                print(f"Video loaded: {file_name}")
                self.enable_controls()
                # Timeline setup
                total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
                self.timeline_slider.setRange(0, total_frames-1)
                self.timeline_slider.setEnabled(True)
                self.total_frames = total_frames
                self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30
                self.timeline_label.setText("00:00 / " + self.format_time(total_frames/self.fps))
                
    def load_image(self):
        file_name, _ = QFileDialog.getOpenFileName(
            self, 
            "Open Image File", 
            "", 
            "Image Files (*.png *.jpg *.jpeg *.bmp)"
        )
        if file_name:
            try:
                # Read the image
                frame = cv2.imread(file_name)
                if frame is None:
                    raise Exception("Failed to load image")
                
                # Process the image
                processed_frame = self.process_frame(frame.copy())
                
                # Convert to RGB for display
                display_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
                
                # Convert to QImage
                h, w, ch = display_frame.shape
                bytes_per_line = ch * w
                qt_image = QImage(display_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
                
                # Display the frame
                self.video_label.setPixmap(QPixmap.fromImage(qt_image).scaled(
                    self.video_label.width(), 
                    self.video_label.height(),
                    Qt.KeepAspectRatio
                ))
                
                # Store the processed frame
                self.processed_frames = [processed_frame]
                
                # Enable controls
                self.enable_controls()
                self.save_image_button.setEnabled(True)
                
            except Exception as e:
                self.show_error_message("Image Loading Error", str(e))

    def enable_controls(self):
        self.play_pause_button.setEnabled(True)
        self.save_button.setEnabled(True)
        self.save_image_button.setEnabled(True)
        self.reset_button.setEnabled(True)
        self.tracking_color_button.setEnabled(True)
        
    def select_color(self, element_type):
        color = QColorDialog.getColor()
        if color.isValid():
            # Convert Qt color to BGR for OpenCV
            bgr_color = (color.blue(), color.green(), color.red())
            
            if element_type == 'tracking':
                self.tracking_color = bgr_color
                self.color_preview.setStyleSheet(
                    f"background-color: rgb({color.red()}, {color.green()}, {color.blue()});"
                )
                self.update_display()
            elif element_type == 'trace':
                self.trace_color = bgr_color
                self.update_display()
            elif element_type == 'coord':
                self.coord_color = bgr_color
                self.update_display()
            elif element_type == 'bbox':
                self.bbox_color = bgr_color
                self.update_display()
            elif element_type == 'marker':
                self.marker_color = bgr_color
                self.update_display()
            elif element_type == 'line':
                self.line_color = bgr_color
                self.update_display()
            elif element_type == 'arrow':
                self.arrow_color = bgr_color
                self.update_display()
                
    def update_display(self):
        """Update the display with current processed frame"""
        if hasattr(self, 'processed_frames') and self.processed_frames:
            frame = self.processed_frames[0].copy()
            processed_frame = self.process_frame(frame)
            display_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
            h, w, ch = display_frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(display_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.video_label.setPixmap(pixmap.scaled(
                self.video_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            ))

    def toggle_play_pause(self):
        if self.cap is not None:
            if self.timer.isActive():
                self.timer.stop()
                self.play_pause_button.setText("Play")
            else:
                speed = self.speed_slider.value() / 100.0
                interval = int(30 / speed)
                self.timer.start(max(1, interval))
                self.play_pause_button.setText("Pause")

    def play_video(self):
        if self.cap is not None:
            try:
                speed = self.speed_slider.value() / 100.0
                interval = int(30 / speed)
                self.timer.start(max(1, interval))
                self.is_recording = True
                self.error_count = 0
                self.play_pause_button.setText("Pause")
            except Exception as e:
                self.show_error_message("Playback Error", str(e))
                self.stop_video()
            
    def update_frame(self):
        if self.cap is not None and not self.is_processing:
            try:
                self.is_processing = True
                ret, frame = self.cap.read()
                if ret:
                    current_frame = int(self.cap.get(cv2.CAP_PROP_POS_FRAMES))
                    processed_frame = self.process_frame(frame.copy())
                    display_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
                    h, w, ch = display_frame.shape
                    bytes_per_line = ch * w
                    qt_image = QImage(display_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
                    pixmap = QPixmap.fromImage(qt_image)
                    self.video_label.setPixmap(pixmap.scaled(
                        self.video_label.size(),
                        Qt.KeepAspectRatio,
                        Qt.SmoothTransformation
                    ))
                    if self.is_recording:
                        self.processed_frames.append(processed_frame)
                    self.timeline_slider.blockSignals(True)
                    self.timeline_slider.setValue(current_frame)
                    self.timeline_slider.blockSignals(False)
                    self.timeline_label.setText(f"{self.format_time(current_frame/self.fps)} / {self.format_time(self.total_frames/self.fps)}")
                    self.error_count = 0
                    if self.is_recording:
                        self.recording_label.setText("Gravando...")
                    else:
                        self.recording_label.setText("")
                else:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            except Exception as e:
                self.error_count += 1
                print(f"Error processing frame: {str(e)}")
                if self.error_count >= self.max_errors:
                    self.show_error_message("Video Processing Error", 
                        "Too many errors occurred. Stopping video playback.")
                    self.stop_video()
                else:
                    self.show_error_message("Frame Processing Error", str(e))
            finally:
                self.is_processing = False
            
    def save_video(self):
        if not self.processed_frames:
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
            self.status_label.setText("Nenhum frame gravado para exportar!")
            self.show_error_message("Save Error", "No frames to save")
            return
        try:
            file_name, _ = QFileDialog.getSaveFileName(
                self,
                "Save Video",
                "",
                "Video Files (*.mp4)"
            )
            if file_name:
                height, width = self.processed_frames[0].shape[:2]
                fps = 30
                fourcc = cv2.VideoWriter_fourcc(*'avc1')
                out = cv2.VideoWriter(file_name, fourcc, fps, (width, height))
                if not out.isOpened():
                    raise Exception("Failed to create video writer")
                for frame in self.processed_frames:
                    out.write(frame)
                out.release()
                print(f"Video saved: {file_name}")
                from PyQt5.QtWidgets import QMessageBox
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Information)
                msg.setWindowTitle("Success")
                msg.setText("Video saved successfully!")
                msg.exec_()
                self.status_label.setStyleSheet("color: green; font-weight: bold;")
                self.status_label.setText("Arquivo salvo com sucesso!")
        except Exception as e:
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
            self.status_label.setText(str(e))
            self.show_error_message("Save Error", str(e))

    def save_image(self):
        if not self.processed_frames:
            self.show_error_message("Save Error", "No image to save")
            return
            
        try:
            file_name, _ = QFileDialog.getSaveFileName(
                self,
                "Save Image",
                "",
                "Image Files (*.png *.jpg *.jpeg *.bmp)"
            )
            
            if file_name:
                # Save the processed frame
                cv2.imwrite(file_name, self.processed_frames[0])
                print(f"Image saved: {file_name}")
                
                # Show success message
                from PyQt5.QtWidgets import QMessageBox
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Information)
                msg.setWindowTitle("Success")
                msg.setText("Image saved successfully!")
                msg.exec_()
                
        except Exception as e:
            self.show_error_message("Save Error", str(e))

    def stop_video(self):
        """Safely stop video playback"""
        try:
            self.timer.stop()
            self.is_recording = False
            if self.cap is not None:
                self.cap.release()
            self.cap = None
            self.play_pause_button.setEnabled(False)
            self.save_button.setEnabled(False)
            self.reset_button.setEnabled(False)
        except Exception as e:
            print(f"Error stopping video: {str(e)}")

    def show_error_message(self, title, message):
        """Show error message to user"""
        from PyQt5.QtWidgets import QMessageBox
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Critical)
        msg.setWindowTitle(title)
        msg.setText(message)
        msg.exec_()
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        self.status_label.setText(message)

    def closeEvent(self, event):
        """Handle application close"""
        try:
            self.stop_video()
            event.accept()
        except Exception as e:
            print(f"Error during close: {str(e)}")
            event.accept()

    def on_tracking_method_changed(self):
        """Handle tracking method change"""
        method = self.tracking_method.currentText()
        if method == "Color Tracking":
            self.hue_range_slider.setEnabled(True)
            self.sat_range_slider.setEnabled(True)
            self.val_range_slider.setEnabled(True)
            self.min_area_spin.setEnabled(True)
        else:
            self.hue_range_slider.setEnabled(False)
            self.sat_range_slider.setEnabled(False)
            self.val_range_slider.setEnabled(False)
            self.min_area_spin.setEnabled(False)
        self.update_display()

    def update_speed(self):
        speed = self.speed_slider.value() / 100.0
        self.speed_label.setText(f"Speed: {speed:.2f}x")
        # Ajusta o timer para nova velocidade
        if self.cap is not None and self.timer.isActive():
            interval = int(30 / speed)
            self.timer.setInterval(max(1, interval))

    def seek_video(self, frame_number):
        if self.cap is not None:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, frame_number)
            ret, frame = self.cap.read()
            if ret:
                processed_frame = self.process_frame(frame.copy())
                display_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
                h, w, ch = display_frame.shape
                bytes_per_line = ch * w
                qt_image = QImage(display_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(qt_image)
                self.video_label.setPixmap(pixmap.scaled(
                    self.video_label.size(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation
                ))
                self.timeline_label.setText(f"{self.format_time(frame_number/self.fps)} / {self.format_time(self.total_frames/self.fps)}")
            # Preview do frame mesmo se não estiver rodando
            if not self.timer.isActive():
                self.play_pause_button.setText("Play")

    def format_time(self, seconds):
        m = int(seconds // 60)
        s = int(seconds % 60)
        return f"{m:02d}:{s:02d}"

    def start_recording(self):
        if self.cap is not None:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        self.is_recording = True
        self.processed_frames = []
        self.start_recording_button.setEnabled(False)
        self.stop_recording_button.setEnabled(True)
        self.save_button.setEnabled(False)

    def stop_recording(self):
        self.is_recording = False
        self.start_recording_button.setEnabled(True)
        self.stop_recording_button.setEnabled(False)
        if self.processed_frames:
            self.save_button.setEnabled(True)

    def resizeEvent(self, event):
        # Atualiza o vídeo ao redimensionar a janela
        self.update_display()
        super().resizeEvent(event)

if __name__ == '__main__':
    print("Starting Tracery application...")
    try:
        app = QApplication(sys.argv)
        print("QApplication created successfully")
        window = TraceryPro()
        print("TraceryPro window created successfully")
        window.show()
        print("Window shown successfully")
        sys.exit(app.exec_())
    except Exception as e:
        print(f"Error occurred: {str(e)}")
        import traceback
        traceback.print_exc()