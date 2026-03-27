from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from kinematics import forward_kinematics, inverse_kinematics
from config import THETA_MIN, THETA_MAX


class ScaraGUI(QWidget):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("SCARA RRR Control Interface")
        self.setGeometry(200, 200, 600, 700)

        self.init_ui()

    def init_ui(self):

        layout = QVBoxLayout()

        # -----------------------
        # SLIDERS (ANGULOS)
        # -----------------------

        grid = QGridLayout()

        self.sliders = []
        self.inputs = []

        for i in range(3):

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(THETA_MIN)
            slider.setMaximum(THETA_MAX)
            slider.setValue(0)

            input_box = QLineEdit("0")

            slider.valueChanged.connect(
                lambda value, box=input_box: box.setText(str(value)))

            grid.addWidget(QLabel(f"θ{i+1} (deg)"), i, 0)
            grid.addWidget(slider, i, 1)
            grid.addWidget(input_box, i, 2)

            self.sliders.append(slider)
            self.inputs.append(input_box)

        layout.addLayout(grid)

        # -----------------------
        # INPUT POSICION XY
        # -----------------------

        xy_layout = QHBoxLayout()

        self.x_input = QLineEdit()
        self.y_input = QLineEdit()

        xy_layout.addWidget(QLabel("X (m):"))
        xy_layout.addWidget(self.x_input)

        xy_layout.addWidget(QLabel("Y (m):"))
        xy_layout.addWidget(self.y_input)

        layout.addLayout(xy_layout)

        # -----------------------
        # BOTONES
        # -----------------------

        btn_layout = QHBoxLayout()

        self.fk_button = QPushButton("Cinemática Directa")
        self.ik_button = QPushButton("Cinemática Inversa")

        self.emergency_button = QPushButton("EMERGENCIA")
        self.emergency_button.setStyleSheet("background-color:red;color:white;")

        btn_layout.addWidget(self.fk_button)
        btn_layout.addWidget(self.ik_button)
        btn_layout.addWidget(self.emergency_button)

        layout.addLayout(btn_layout)

        # -----------------------
        # RESULTADOS
        # -----------------------

        self.position_label = QLabel("Posición XY:")
        layout.addWidget(self.position_label)

        self.console = QTextEdit()
        self.console.setReadOnly(True)

        layout.addWidget(QLabel("Log del sistema"))
        layout.addWidget(self.console)

        self.setLayout(layout)

        # -----------------------
        # CONEXIONES
        # -----------------------

        self.fk_button.clicked.connect(self.calculate_fk)
        self.ik_button.clicked.connect(self.calculate_ik)
        self.emergency_button.clicked.connect(self.emergency)

    # --------------------------------
    # CINEMATICA DIRECTA
    # --------------------------------

    def calculate_fk(self):

        try:
            theta = [float(box.text()) for box in self.inputs]

            x, y = forward_kinematics(*theta)

            self.position_label.setText(f"Posición XY: x={x} m , y={y} m")

            self.console.append(
                f"FK → θ={theta} → XY=({x},{y})"
            )

        except:
            self.console.append("Error en FK")

    # --------------------------------
    # CINEMATICA INVERSA
    # --------------------------------

    def calculate_ik(self):

        try:

            x = float(self.x_input.text())
            y = float(self.y_input.text())

            t1, t2, t3 = inverse_kinematics(x, y)

            self.sliders[0].setValue(int(t1))
            self.sliders[1].setValue(int(t2))
            self.sliders[2].setValue(int(t3))

            self.console.append(
                f"IK → XY=({x},{y}) → θ=({t1},{t2},{t3})"
            )

        except:
            self.console.append("Error en IK o punto fuera de alcance")

    # --------------------------------
    # EMERGENCIA
    # --------------------------------

    def emergency(self):

        for slider in self.sliders:
            slider.setValue(0)

        self.console.append("⚠ Emergencia activada")