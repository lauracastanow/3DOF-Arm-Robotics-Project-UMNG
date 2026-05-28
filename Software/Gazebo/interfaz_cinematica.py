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
                             QLineEdit, QComboBox, QFormLayout, QMessageBox)
from PyQt5.QtCore import Qt, QTimer

class RobotInterface(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Control Maestro Ensamblee - Trayectorias")
        self.resize(1400, 900)
        
        # --- MEDIDAS DEL URDF (mm) ---
        self.L_base_fija = 195.0   
        self.L_inferior = 180.0  
        self.L_superior = 125.0  

        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node('interfaz_robot_node')
        
        self.pub_j1 = self.node.create_publisher(Float64, '/model/Ensamblee/joint/joint_1/cmd_pos', 10)
        self.pub_j2 = self.node.create_publisher(Float64, '/model/Ensamblee/joint/joint_2/cmd_pos', 10)
        self.pub_j3 = self.node.create_publisher(Float64, '/model/Ensamblee/joint/joint_3/cmd_pos', 10)

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

        # --- SECCIÓN 2: CINEMÁTICA INVERSA Y TRAYECTORIAS ---
        controls.addSpacing(20)
        controls.addWidget(QLabel("<b>PLANEACIÓN DE TRAYECTORIAS (IK)</b>"))
        
        form_layout = QFormLayout()
        self.input_x = QLineEdit("150"); self.input_y = QLineEdit("150"); self.input_z = QLineEdit("200")
        self.input_tf = QLineEdit("5.0") # Tiempo total en segundos
        
        self.combo_config = QComboBox()
        self.combo_config.addItems(["Codo Arriba", "Codo Abajo"])
        
        form_layout.addRow("Posición X (mm):", self.input_x)
        form_layout.addRow("Posición Y (mm):", self.input_y)
        form_layout.addRow("Posición Z (mm):", self.input_z)
        form_layout.addRow("Tiempo de viaje tf (s):", self.input_tf)
        form_layout.addRow("Configuración:", self.combo_config)
        
        controls.addLayout(form_layout)
        
        self.btn_trayectoria = QPushButton("GENERAR Y EJECUTAR TRAYECTORIA")
        self.btn_trayectoria.setStyleSheet("background-color: #0066cc; color: white; font-weight: bold; padding: 10px;")
        self.btn_trayectoria.clicked.connect(self.iniciar_trayectoria)
        controls.addWidget(self.btn_trayectoria)

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
        # Desactivamos el trigger automático durante la simulación para evitar bugs
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
        """Cálculo estricto de Cinemática Inversa con conversión a Gazebo"""
        L1, L2, L3 = self.L_base_fija, self.L_inferior, self.L_superior
        r = np.sqrt(x**2 + y**2)
        
        c3 = (x**2 + y**2 + (z - L1)**2 - L2**2 - L3**2) / (2 * L2 * L3)
        if abs(c3) > 1:
            raise ValueError("El punto está fuera del espacio de trabajo del robot.")
            
        # Elegir rama de la solución
        if config == "Codo Arriba":
            s3 = np.sqrt(1 - c3**2)
        else:
            s3 = -np.sqrt(1 - c3**2)
            
        th3 = np.arctan2(s3, c3)
        th2 = np.arctan2(z - L1, r) - np.arctan2(L3 * np.sin(th3), L2 + L3 * np.cos(th3))
        th1 = np.arctan2(y, x)
        
        # --- CONVERSIÓN DE ÁNGULOS PUROS A ÁNGULOS GAZEBO ---
        # Garantiza que el planificador de trayectorias mueva los sliders y 
        # Gazebo al punto correcto considerando la posición "Home" en forma de L.
        q1_gaz = th1
        q2_gaz = (np.pi / 2) - th2
        q3_gaz = -th3 - (np.pi / 2)
        
        return q1_gaz, q2_gaz, q3_gaz

    def iniciar_trayectoria(self):
        try:
            x = float(self.input_x.text())
            y = float(self.input_y.text())
            z = float(self.input_z.text())
            tf = float(self.input_tf.text())
            config = self.combo_config.currentText()
            
            # 1. Obtener destino (IK)
            q_f = self.calcular_ik(x, y, z, config)
            self.P_objetivo = [x, y, z]
            
            # 2. Obtener origen actual (q0)
            q_0 = [np.radians(self.s1.value()), 
                   np.radians(self.s2.value()), 
                   np.radians(self.s3.value())]
                   
            # 3. Generar polinomio cúbico (Perfil de velocidad suave)
            t_resolucion = 0.05 # 50 ms por paso (20 Hz)
            n_pasos = int(tf / t_resolucion)
            self.t_traj = np.linspace(0, tf, n_pasos)
            
            self.q_traj = np.zeros((3, n_pasos))
            self.dq_traj = np.zeros((3, n_pasos))
            
            for i in range(3):
                # Coeficientes del polinomio q(t) = a0 + a2*t^2 + a3*t^3
                # Garantiza que velocidad en t=0 y t=tf sea 0.
                a0 = q_0[i]
                a2 = 3 * (q_f[i] - q_0[i]) / (tf**2)
                a3 = -2 * (q_f[i] - q_0[i]) / (tf**3)
                
                self.q_traj[i, :] = a0 + a2 * (self.t_traj**2) + a3 * (self.t_traj**3)
                self.dq_traj[i, :] = 2 * a2 * self.t_traj + 3 * a3 * (self.t_traj**2)
                
            # 4. Iniciar timer para la animación
            self.paso_actual = 0
            self.btn_trayectoria.setEnabled(False)
            self.timer.start(int(t_resolucion * 1000)) # Milisegundos
            
        except ValueError as e:
            QMessageBox.warning(self, "Error de Cinemática", str(e))
        except Exception as e:
            QMessageBox.critical(self, "Error de Input", "Por favor, verifique que los valores ingresados sean numéricos.")

    def ejecutar_paso_trayectoria(self):
        if self.paso_actual < len(self.t_traj):
            # Extraer posición actual y actualizar sliders (para reflejar en la GUI visualmente)
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
            
            # Forzar actualización
            self.update_robot()
            
            self.paso_actual += 1
        else:
            self.timer.stop()
            self.btn_trayectoria.setEnabled(True)
            self.mostrar_graficas_finales()

    def update_robot(self):
        # 1. Obtener ángulos (AQUÍ AHORA SON ÁNGULOS GAZEBO PUROS)
        deg1, deg2, deg3 = self.s1.value(), self.s2.value(), self.s3.value()
        q1, q2, q3 = np.radians(deg1), np.radians(deg2), np.radians(deg3)

        # 2. Convertir Ángulos Gazebo a Ángulos DH (Para que el dibujo 3D coincida)
        # Gazebo en 0 = Hombro arriba, Codo al frente.
        # Desfasamos e invertimos ejes matemáticos para igualar la realidad física
        q1_dh = q1
        q2_dh = -q2 + (np.pi / 2)
        q3_dh = -q3 - (np.pi / 2)

        # 3. Calcular Cinemática con los ángulos DH
        T01 = self.dh(q1_dh, self.L_base_fija, 0, np.pi/2) 
        T12 = self.dh(q2_dh, 0, self.L_inferior, 0)        
        T23 = self.dh(q3_dh, 0, self.L_superior, 0)        
        
        T02 = T01 @ T12
        T03 = T02 @ T23 # Matriz del Efector Final

        # 4. Extraer Posición XYZ
        pos_x = np.round(T03[0, 3], 2)
        pos_y = np.round(T03[1, 3], 2)
        pos_z = np.round(T03[2, 3], 2)

        # 5. Actualizar Monitor
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

        # 6. Publicar a Gazebo (DIRECTO, sin multiplicar por -1)
        m1, m2, m3 = Float64(), Float64(), Float64()
        m1.data, m2.data, m3.data = float(q1), float(q2), float(q3)
        self.pub_j1.publish(m1); self.pub_j2.publish(m2); self.pub_j3.publish(m3)

        # 7. Dibujo 3D
        self.ax.clear()
        p0, p1 = [0,0,0], [0,0, self.L_base_fija]
        p2, p3, p4 = T01[:3, 3], T02[:3, 3], T03[:3, 3]

        self.ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], color='black', linewidth=3)
        self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color='red', linewidth=6)   
        self.ax.plot([p2[0], p3[0]], [p2[1], p3[1]], [p2[2], p3[2]], color='blue', linewidth=4)  
        self.ax.plot([p3[0], p4[0]], [p3[1], p4[1]], [p3[2], p4[2]], color='green', linewidth=4) 

        # Dibujar punto objetivo si existe
        if self.P_objetivo is not None:
            self.ax.plot([self.P_objetivo[0]], [self.P_objetivo[1]], [self.P_objetivo[2]], 'x', color='orange', markersize=10, markeredgewidth=3)

        self.ax.set_xlim(-400, 400); self.ax.set_ylim(-400, 400); self.ax.set_zlim(0, 500)
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')
        self.canvas.draw()
        rclpy.spin_once(self.node, timeout_sec=0)
        
        
    def mostrar_graficas_finales(self):
        """Muestra las gráficas usando Matplotlib en una ventana externa tras finalizar la trayectoria"""
        fig_plots, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
        fig_plots.canvas.manager.set_window_title('Análisis de Trayectoria')

        # Gráfica de Posición
        ax1.plot(self.t_traj, np.degrees(self.q_traj[0, :]), 'r', label='Q1 (Base)', linewidth=2)
        ax1.plot(self.t_traj, np.degrees(self.q_traj[1, :]), 'b', label='Q2 (Hombro)', linewidth=2)
        ax1.plot(self.t_traj, np.degrees(self.q_traj[2, :]), 'g', label='Q3 (Codo)', linewidth=2)
        ax1.set_title('Posición Articular vs Tiempo')
        ax1.set_ylabel('Posición (Grados)')
        ax1.grid(True)
        ax1.legend()

        # Gráfica de Velocidad
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
