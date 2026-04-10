# 3DOF-Arm-Robotics-Project-UMNG
Este proyecto consiste en el diseño, modelado y simulación de un brazo robótico de 3 grados de libertad. En el siguiente repositorio, se incluye el desarrollo mecánico en CAD, simulación en software, código de control y una interfaz interactiva para visualizar la cinemática inversa. 

## Objetivos
### Objetivo General
Diseñar un brazo robótico integrando herramientas de hardware (CAD) y software (ROS 2, Gazebo, Python) para la posición del efector final mediante el cálculo de la cinemática y control de los actuadores en tiempo real.

### Objetivos Específicos
* Diseñar la estructura mecánica del brazo y su exportación a formato URDF.
* Modelar matemáticamente el sistema para implementar cinemática directa e inversa.
* Configurar con la simulación física en Gazebo Sim (Dynamics, colisiones e inercias).
* Establecer comunicación en tiempo real entre ROS 2 y Gazebo usando 'ros_gz_bridge'.
* Desarrollar una interfaz gráfica de usuario (GUI) interactiva para el control articular.

---

## Estructura del repositorio
El proyecto está modularizado basándose en el diagrama de bloques del sistema:

* 'Docs/' Diagramas de bloques, actas de trabajo y capturas de funcionamiento.
* 'Hardware/' Diseño CAD, mallas (meshes) y el archivo de descripción 'Ensamblee.urdf'.
* 'Simulación/' Archivos de lanzamiento ('gazebo.launch.py') y configuración del entorno virtual.
* 'Software/' Lógica de control, cinemática ('kinematics.py'), y nodo de ROS 2 conectado a la GUI ('gui.py')

---

## Descripción del sistema
El robot ("Bracito") es un manipulador que cuenta con los siguientes grados de libertad controlables:

* **Joint 1 (Base):** Rotación sobre el eje Z.
* **Joint 2 (Hombro):** Elevación del primer eslabón.
* **Joint 3 (Codo):** Elevación del segundo eslabón.
* **Joints 4, 5 y 6:** Pinza de 3 dedos configurada con plugins *Mimic* en Gazebo, permitiendo que la actuación de un solo motor cierre o abra la pinza completa de manera síncrona.

![Ensamblaje General SolidWorks](Docs/Ensamblaje General SW.png)

El modelo permite calcular:
* Cinemática inversa
* Posición del efector final
![Interfaz de Control Maestro y Gazebo funcionando](Simulación/Simulacion_Gazebo_interfaz.jpg)

---

## Paquetes, Dependencias y Librerías
Para la correcta ejecución de etse proyecto, es necesario contal con el siguiente entorno:

**Entorno Base:**
* Ubuntu 24.04 (o compatible)
* [ROS 2 Jazzy] (https://docs.ros.org/en/jazzy/index.html)
*  Gazebo Sim (Harmonic)

**Librerías de Python:**
* 'rclpy' (Cliente de ROS 2 para Python)
* 'PyQt5' (Desarrollo e la interfaz gráfica)
* 'numpy' (Cálculos matriciales para la cinemática)

**Paquetes de ROS 2 utilizados:**
* 'robot_state_publisher': Para publicar el estado del robot basado en el URDF.
* 'ros_gz_sim': Para instanciar (spawn) el modelo en Gazebo.
* 'ros_gz_bridge': Para traducir los tópicos de 'JointState' / 'Float64' entre ROS 2 y Gazebo.

---

## Autoras
Proyecto desarrollado por estudiantes de Ingeniería de la Universidad Militar Nueva Granada (UMNG):
* **Laura Daniela Castaño Silva**
* **Carol Tatiana Jeréz González**
