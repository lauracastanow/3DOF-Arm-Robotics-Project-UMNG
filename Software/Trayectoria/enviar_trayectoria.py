import serial
import time
import math
import os

# === 1. CONFIGURACIÓN IMPORTANTE ===
PUERTO = 'COM3' 
BAUDIOS = 115200
NOMBRE_ARCHIVO = 'Brazolauracarol.txt' 

# === EL TRUCO DE MAGIA PARA LAS CARPETAS ===
# Esto obliga a Python a buscar en la misma carpeta del script, pase lo que pase.
directorio_script = os.path.dirname(os.path.abspath(__file__))
ruta_completa = os.path.join(directorio_script, NOMBRE_ARCHIVO)

try:
    print(f"1. Intentando conectar al ESP32 en el puerto {PUERTO}...")
    esp32 = serial.Serial(PUERTO, BAUDIOS)
    time.sleep(2) 
    print("2. ¡ESP32 Conectado exitosamente!")
    
    print(f"3. Abriendo el archivo exactamente en:\n   {ruta_completa}")
    with open(ruta_completa, 'r') as file:
        next(file) # Saltar la primera línea (t,q1,q2,q3)
        
        print("4. ¡Atención! Iniciando movimiento en 3 segundos...")
        time.sleep(3)
        
        for linea in file:
            valores = linea.strip().split(',')
            if len(valores) == 4:
                t = float(valores[0])
                q1_rad = float(valores[1])
                
                # Convertimos tu radian a grados
                q1_deg = q1_rad * (180.0 / math.pi)
                
                # Le enviamos el grado al ESP32
                esp32.write(f"{q1_deg}\n".encode())
                print(f"Tiempo: {t}s -> Enviando M1: {q1_deg:.2f}°")
                
                # Respetamos el salto de tu simulador (0.05 segundos)
                time.sleep(0.05)
                
    print("\n¡Trayectoria terminada con éxito!")
    esp32.close()

except FileNotFoundError:
    print(f"\n[ERROR] Sigo sin encontrar el archivo.")
    print("¿Posible solución? Revisa que Windows no lo haya guardado como 'Brazolauracarol.txt.txt'.")
    print("Intenta renombrar el archivo en tu carpeta a solo 'Brazolauracarol' (sin el .txt al final).")
except serial.SerialException:
    print(f"\n[ERROR] El puerto {PUERTO} está bloqueado o no existe.")
except Exception as e:
    print(f"\n[ERROR DESCONOCIDO]: {e}")

input("\nPresiona ENTER para salir...")