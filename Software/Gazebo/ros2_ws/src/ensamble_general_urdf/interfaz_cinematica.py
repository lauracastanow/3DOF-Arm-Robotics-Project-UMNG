import sys
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QSlider, QLabel, QTextEdit, QPushButton,
                             QComboBox, QFormLayout, QMessageBox)
from PyQt5.QtCore import Qt, QTimer

class RobotInterface(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Control Maestro Ensamble - Secuencia Trapezoidal")
        self.resize(1400, 900)
        
        # --- MEDIDAS DEL URDF (mm) ---
        # ¡AQUÍ AJUSTAMOS LO DE LAS ESTIBAS!
        self.altura_estibas = 114.0  # Cambia esto por la altura real de tus estibas en mm
        self.distancia_motor_2 = 126.5 # Lo que nos dio el URDF
        
        self.L_base_fija = self.altura_estibas + self.distancia_motor_2   
        self.L_inferior = 180.0  
        self.L_superior = 125.0  

        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node('interfaz_robot_node')
        
        # --- TÓPICOS ACTUALIZADOS ---
        self.pub_j1 = self.node.create_publisher(Float64, '/model/ensamble_general_urdf/joint/joint_1/cmd_pos', 10)
        self.pub_j2 = self.node.create_publisher(Float64, '/model/ensamble_general_urdf/joint/joint_2/cmd_pos', 10)
        self.pub_j3 = self.node.create_publisher(Float64, '/model/ensamble_general_urdf/joint/joint_3/cmd_pos', 10)

        # Variables para la trayectoria
        self.timer = QTimer()
        self.timer.timeout.connect(self.ejecutar_paso_trayectoria)
        self.paso_actual = 0
        self.q_traj = []
        self.dq_traj = []
        self.t_traj = []
        self.P_objetivo = None

        self.initUI()

    def initUI(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)
        
        # PANEL IZQUIERDO: CONTROLES Y MONITOR
        controls = QVBoxLayout()
        
        # --- SECCIÓN 1: CONTROL MANUAL ---
        controls.addWidget(QLabel("<b>CONTROL ARTICULAR DIRECTO</b>"))
        self.s1 = self.create_slider("M1 - Base Giratoria (Rojo):", controls)
        self.s2 = self.create_slider("M2 - Brazo Largo (Azul):", controls)
        self.s3 = self.create_slider("M3 - Brazo Corto (Verde):", controls)
        
        self.btn_reset = QPushButton("RESET A CERO")
        self.btn_reset.setStyleSheet("background-color: #444; color: white; font-weight: bold;")
        self.btn_reset.clicked.connect(self.reset_positions)
        controls.addWidget(self.btn_reset)

        # --- SECCIÓN 2: CINEMÁTICA INVERSA Y SECUENCIAS ---
        controls.addSpacing(20)
        controls.addWidget(QLabel("<b>SECUENCIA DE OPERACIÓN (TRAPEZOIDAL)</b>"))
        
        form_layout = QFormLayout()
        self.combo_config = QComboBox()
        self.combo_config.addItems(["Codo Abajo"])
        form_layout.addRow("Configuración IK:", self.combo_config)
        controls.addLayout(form_layout)
        
        # Botones de secuencia
        self.btn_home = QPushButton("1. Ir a HOME (Reposo)")
        self.btn_home.setStyleSheet("background-color: #555; color: white; padding: 10px; font-weight: bold;")
        self.btn_home.clicked.connect(lambda: self.ejecutar_movimiento("HOME", 150, 0, 200))

        # OJO: Coordenadas de prueba. Probablemente toque ajustarlas con la nueva altura.
        self.btn_pick = QPushButton("2. Ir a RECOGER Ping-Pong")
        self.btn_pick.setStyleSheet("background-color: #d9534f; color: white; padding: 10px; font-weight: bold;")
        self.btn_pick.clicked.connect(lambda: self.ejecutar_movimiento("PICK", 78.26, 160.45, 234.90)) 

        self.btn_place = QPushButton("3. Ir a DEJAR (90° Horario)")
        self.btn_place.setStyleSheet("background-color: #5cb85c; color: white; padding: 10px; font-weight: bold;")
        self.btn_place.clicked.connect(lambda: self.ejecutar_movimiento("PLACE", 152.70, -88.16, 201.22)) 
        
        self.btn_place2 = QPushButton("4. Ir a DEJAR pinpon 2")
        self.btn_place2.setStyleSheet("background-color: #5cb85c; color: white; padding: 10px; font-weight: bold;")
        self.btn_place2.clicked.connect(lambda: self.ejecutar_movimiento("PLACE", 175.0, -28.00, 201.22))
        
        self.btn_place3 = QPushButton("5. Ir a DEJAR pinpon 3")
        self.btn_place3.setStyleSheet("background-color: #5cb85c; color: white; padding: 10px; font-weight: bold;")
        self.btn_place3.clicked.connect(lambda: self.ejecutar_movimiento("PLACE", 173.00, 34.00, 201.22))

        controls.addWidget(self.btn_home)
        controls.addWidget(self.btn_pick)
        controls.addWidget(self.btn_place)
        controls.addWidget(self.btn_place2)
        controls.addWidget(self.btn_place3)

        self.lbl_tiempo = QLabel("<b>Tiempo de ejecución estimado:</b> -- s")
        self.lbl_tiempo.setStyleSheet("color: #0066cc; font-size: 14px; margin-top: 10px;")
        controls.addWidget(self.lbl_tiempo)

        # --- SECCIÓN 3: MONITOR DE DATOS ---
        controls.addSpacing(20)
        controls.addWidget(QLabel("<b>TELEMETRÍA Y CINEMÁTICA:</b>"))
        self.monitor = QTextEdit()
        self.monitor.setReadOnly(True)
        self.monitor.setStyleSheet("""
            background-color: black; 
            color: #33FF33; 
            font-family: 'Courier New'; 
            font-size: 11pt;
            border: 2px solid #555;
        """)
        controls.addWidget(self.monitor)
        layout.addLayout(controls, 1)

        # PANEL DERECHO (3D)
        self.fig = Figure()
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111, projection='3d')
        layout.addWidget(self.canvas, 2)
        
        self.update_robot()

    def create_slider(self, name, layout):
        layout.addWidget(QLabel(f"<b>{name}</b>"))
        s = QSlider(Qt.Horizontal)
        s.setRange(-180, 180); s.setValue(0)
        s.valueChanged.connect(self.slider_manual_update)
        layout.addWidget(s)
        return s

    def slider_manual_update(self):
        if not self.timer.isActive():
            self.update_robot()

    def reset_positions(self):
        self.P_objetivo = None
        self.s1.setValue(0); self.s2.setValue(0); self.s3.setValue(0)
        self.update_robot()

    def dh(self, theta, d, a, alpha):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),               np.cos(alpha),              d],
            [0,              0,                           0,                          1]
        ])

    def calcular_ik(self, x, y, z, config):
        L1, L2, L3 = self.L_base_fija, self.L_inferior, self.L_superior
        r = np.sqrt(x**2 + y**2)
        
        c3 = (x**2 + y**2 + (z - L1)**2 - L2**2 - L3**2) / (2 * L2 * L3)
        if abs(c3) > 1:
            raise ValueError("El punto está fuera del espacio de trabajo del robot.")
            
        if config == "Codo Arriba":
            s3 = np.sqrt(1 - c3**2)
        else:
            s3 = -np.sqrt(1 - c3**2)
            
        th3 = np.arctan2(s3, c3)
        th2 = np.arctan2(z - L1, r) - np.arctan2(L3 * np.sin(th3), L2 + L3 * np.cos(th3))
        th1 = np.arctan2(y, x)
        
        q1_gaz = th1
        q2_gaz = (np.pi / 2) - th2
        q3_gaz = -th3 - (np.pi / 2)
        
        return q1_gaz, q2_gaz, q3_gaz

    def calcular_trapezoidal_inteligente(self, q_0, q_f):
        distancias = np.abs(np.array(q_f) - np.array(q_0))
        D_max = np.max(distancias)
        V_max = 1.8 
        
        if D_max < 0.001:
            tf = 1.0 
        else:
            tf = (1.5 * D_max) / V_max

        t_resolucion = 0.05
        n_pasos = int(np.ceil(tf / t_resolucion))
        tf_real = n_pasos * t_resolucion

        self.t_traj = np.linspace(0, tf_real, n_pasos)
        self.q_traj = np.zeros((3, n_pasos))
        self.dq_traj = np.zeros((3, n_pasos)) 
        
        tb = tf_real / 3.0  

        for i in range(3):
            D = q_f[i] - q_0[i] 
            if abs(D) < 1e-6:
                self.q_traj[i, :] = q_0[i]
                continue  
            
            V_crucero = 1.5 * D / tf_real  
            a = V_crucero / tb        
            
            for k, t in enumerate(self.t_traj):
                if t < tb:
                    self.q_traj[i, k] = q_0[i] + 0.5 * a * t**2
                    self.dq_traj[i, k] = a * t
                elif t <= (tf_real - tb):
                    self.q_traj[i, k] = q_0[i] + 0.5 * a * tb**2 + V_crucero * (t - tb)
                    self.dq_traj[i, k] = V_crucero     
                else:
                    t_freno = tf_real - t 
                    self.q_traj[i, k] = q_f[i] - 0.5 * a * t_freno**2
                    self.dq_traj[i, k] = a * t_freno         
        
        return tf_real

    def ejecutar_movimiento(self, nombre_fase, x, y, z):
        try:
            config = self.combo_config.currentText()
            
            q_0 = [np.radians(self.s1.value()), 
                   np.radians(self.s2.value()), 
                   np.radians(self.s3.value())]

            if nombre_fase == "HOME":
                q_f = [0.0, 0.0, 0.0]
                self.P_objetivo = None
            else:
                q_f = self.calcular_ik(x, y, z, config)
                self.P_objetivo = [x, y, z]

            tiempo_estimado = self.calcular_trapezoidal_inteligente(q_0, q_f)
            
            self.lbl_tiempo.setText(f"<b>Fase actual:</b> {nombre_fase}<br><b>Tiempo de ejecución estimado:</b> {tiempo_estimado:.2f} segundos")

            self.btn_home.setEnabled(False)
            self.btn_pick.setEnabled(False)
            self.btn_place.setEnabled(False)
            
            self.paso_actual = 0
            self.timer.start(50) 
            
        except ValueError as e:
            QMessageBox.warning(self, "Error de Alcance", f"Esa posición no es alcanzable para el robot.\n\nDetalle: {str(e)}")
        except Exception as e:
            QMessageBox.critical(self, "Error Fatal", str(e))

    def ejecutar_paso_trayectoria(self):
        if self.paso_actual < len(self.t_traj):
            q_actual = self.q_traj[:, self.paso_actual]
            
            self.s1.blockSignals(True)
            self.s2.blockSignals(True)
            self.s3.blockSignals(True)
            
            self.s1.setValue(int(np.degrees(q_actual[0])))
            self.s2.setValue(int(np.degrees(q_actual[1])))
            self.s3.setValue(int(np.degrees(q_actual[2])))
            
            self.s1.blockSignals(False)
            self.s2.blockSignals(False)
            self.s3.blockSignals(False)
            
            self.update_robot()
            self.paso_actual += 1
        else:
            self.timer.stop()
            self.btn_home.setEnabled(True)
            self.btn_pick.setEnabled(True)
            self.btn_place.setEnabled(True)
            self.mostrar_graficas_finales()

    def update_robot(self):
        deg1, deg2, deg3 = self.s1.value(), self.s2.value(), self.s3.value()
        q1, q2, q3 = np.radians(deg1), np.radians(deg2), np.radians(deg3)

        q1_dh = q1
        q2_dh = -q2 + (np.pi / 2)
        q3_dh = -q3 - (np.pi / 2)

        T01 = self.dh(q1_dh, self.L_base_fija, 0, np.pi/2) 
        T12 = self.dh(q2_dh, 0, self.L_inferior, 0)        
        T23 = self.dh(q3_dh, 0, self.L_superior, 0)        
        
        T02 = T01 @ T12
        T03 = T02 @ T23

        pos_x = np.round(T03[0, 3], 2)
        pos_y = np.round(T03[1, 3], 2)
        pos_z = np.round(T03[2, 3], 2)

        texto_monitor = f"""--- ÁNGULOS GAZEBO ACTUALES ---
Q1: {deg1}° | Q2: {deg2}° | Q3: {deg3}°

--- POSICIÓN EFECTOR (mm) ---
X: {pos_x}
Y: {pos_y}
Z: {pos_z}

--- MATRIZ HOMOGÉNEA T03 ---
{np.array2string(np.round(T03, 3), separator=', ')}
"""
        self.monitor.setText(texto_monitor)

        m1, m2, m3 = Float64(), Float64(), Float64()
        m1.data, m2.data, m3.data = float(q1), float(q2), float(q3)
        self.pub_j1.publish(m1); self.pub_j2.publish(m2); self.pub_j3.publish(m3)

        self.ax.clear()
        p0, p1 = [0,0,0], [0,0, self.L_base_fija]
        p2, p3, p4 = T01[:3, 3], T02[:3, 3], T03[:3, 3]

        self.ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], color='black', linewidth=3)
        self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color='red', linewidth=6)   
        self.ax.plot([p2[0], p3[0]], [p2[1], p3[1]], [p2[2], p3[2]], color='blue', linewidth=4)  
        self.ax.plot([p3[0], p4[0]], [p3[1], p4[1]], [p3[2], p4[2]], color='green', linewidth=4) 

        if self.P_objetivo is not None:
            self.ax.plot([self.P_objetivo[0]], [self.P_objetivo[1]], [self.P_objetivo[2]], 'x', color='orange', markersize=10, markeredgewidth=3)

        self.ax.set_xlim(-400, 400); self.ax.set_ylim(-400, 400); self.ax.set_zlim(0, 500)
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')
        self.canvas.draw()
        rclpy.spin_once(self.node, timeout_sec=0)
        
    def mostrar_graficas_finales(self):
        fig_plots, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
        fig_plots.canvas.manager.set_window_title('Análisis de Trayectoria')

        ax1.plot(self.t_traj, np.degrees(self.q_traj[0, :]), 'r', label='Q1 (Base)', linewidth=2)
        ax1.plot(self.t_traj, np.degrees(self.q_traj[1, :]), 'b', label='Q2 (Hombro)', linewidth=2)
        ax1.plot(self.t_traj, np.degrees(self.q_traj[2, :]), 'g', label='Q3 (Codo)', linewidth=2)
        ax1.set_title('Posición Articular vs Tiempo')
        ax1.set_ylabel('Posición (Grados)')
        ax1.grid(True)
        ax1.legend()

        ax2.plot(self.t_traj, self.dq_traj[0, :], 'r', label='dQ1', linewidth=2)
        ax2.plot(self.t_traj, self.dq_traj[1, :], 'b', label='dQ2', linewidth=2)
        ax2.plot(self.t_traj, self.dq_traj[2, :], 'g', label='dQ3', linewidth=2)
        ax2.set_title('Velocidad Articular vs Tiempo')
        ax2.set_xlabel('Tiempo (s)')
        ax2.set_ylabel('Velocidad (rad/s)')
        ax2.grid(True)
        ax2.legend()

        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = RobotInterface()
    gui.show()
    sys.exit(app.exec_())
