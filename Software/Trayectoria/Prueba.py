import serial
import time
import math

# 1. Configura el puerto donde está tu ESP32 (Ej: 'COM3' en Windows o '/dev/ttyUSB0' en Mac/Linux)
PUERTO = 'COM3' 
BAUDIOS = 115200

try:
    print(f"Conectando al ESP32 en {PUERTO}...")
    esp32 = serial.Serial(PUERTO, BAUDIOS)
    time.sleep(2) # Esperar a que el ESP32 reinicie la conexión
    print("Conectado. Iniciando streaming de trayectoria...")
    
    with open('Brazolauracarol.txt', 'r') as file:
        next(file) # Saltamos la primera línea (el encabezado t,q1,q2,q3)
        
        for linea in file:
            valores = linea.strip().split(',')
            if len(valores) == 4:
                t = float(valores[0])
                q1_rad = float(valores[1])
                
                # Convertir radianes a grados
                q1_deg = q1_rad * (180.0 / math.pi)
                
                # Enviar el ángulo al ESP32 terminado en un salto de línea
                esp32.write(f"{q1_deg}\n".encode())
                
                print(f"Tiempo: {t}s -> Enviado M1: {q1_deg:.2f}°")
                
                # El archivo tiene incrementos de 0.05s (50ms). Respetamos esa velocidad.
                time.sleep(0.05)
                
    print("¡Trayectoria de Gazebo enviada con éxito!")
    esp32.close()

except Exception as e:
    print(f"Error de conexión o lectura: {e}")