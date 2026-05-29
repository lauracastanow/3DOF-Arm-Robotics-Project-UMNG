import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import csv

class ReproductorTrayectoriaSimple(Node):
    def __init__(self):
        super().__init__('reproductor_trayectoria')
        # Tópicos de los joints
        self.pub1 = self.create_publisher(Float64, '/model/ensamble_general_urdf/joint/joint_1/cmd_pos', 10)
        self.pub2 = self.create_publisher(Float64, '/model/ensamble_general_urdf/joint/joint_2/cmd_pos', 10)
        self.pub3 = self.create_publisher(Float64, '/model/ensamble_general_urdf/joint/joint_3/cmd_pos', 10)
        
        self.trayectoria = []
        self.idx = 0
        self.ruta = 'Brazolauracarol.txt'
        
        self.cargar_archivo()
        # Timer ajustado a 0.05s para que coincida con el dt de MATLAB
        self.timer = self.create_timer(0.05, self.publicar)

    def cargar_archivo(self):
        try:
            with open(self.ruta, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                next(reader) # Saltar encabezado
                for fila in reader:
                    if len(fila) >= 4:
                        # Leemos los ángulos que YA vienen convertidos de MATLAB
                        self.trayectoria.append([float(fila[1]), float(fila[2]), float(fila[3])])
            self.get_logger().info(f'Trayectoria cargada: {len(self.trayectoria)} puntos.')
        except Exception as e:
            self.get_logger().error(f'Error al abrir: {e}')

    def publicar(self):
        if self.idx < len(self.trayectoria):
            q1, q2, q3 = self.trayectoria[self.idx]
            
            # Mandamos directo, sin matemáticas extra
            self.pub1.publish(Float64(data=q1))
            self.pub2.publish(Float64(data=q2))
            self.pub3.publish(Float64(data=q3))
            
            self.idx += 1
        else:
            self.get_logger().info('¡Ciclo de pinpones completado!')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ReproductorTrayectoriaSimple())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
