#!/usr/bin/env python3
"""
Servidor Flask para el control del Robot RRP mediante cinemática inversa,
cinemática directa, control de posición acumulada y comunicación serial con el ESP32.
"""

import threading
import time
import numpy as np
from flask import Flask, jsonify, request, send_from_directory
import serial
from serial.tools import list_ports

app = Flask(__name__, static_folder='.', template_folder='.')

# Variables globales de conexión
conexion_serial = None
hilo_lector = None
detener_hilo = threading.Event()
puerto_actual = None
baudrate_actual = 115200

# Longitudes de los eslabones del robot RRP (en cm)
L1 = 10.0
L2 = 10.0

# Estado actual del robot (se asume en 0 al conectar)
# Se almacenan en radianes para q1 y q2, y en cm para q3 (Z)
current_q1 = 0.0
current_q2 = 0.0
current_q3 = 0.0


def cinematica_inversa(x, y, z, l1, l2, codo="abajo"):
    """
    Cinemática inversa del robot RRP (DH: [q1 0 l1 0; q2 0 l2 0; 0 q3 0 0]).
    Basado en los módulos cinemáticos del proyecto[cite: 3, 4].
    """
    q3 = z
    r2 = x**2 + y**2
    if r2 == 0:
        raise ValueError("Las coordenadas X e Y no pueden ser ambas 0 (singularidad).")
    
    cos_q2 = (r2 - l1**2 - l2**2) / (2 * l1 * l2)

    if abs(cos_q2) > 1 + 1e-9:
        raise ValueError(
            f"Punto fuera del espacio de trabajo: "
            f"{abs(l1 - l2):.3f} <= sqrt(x²+y²) <= {l1 + l2:.3f}"
        )
    cos_q2 = np.clip(cos_q2, -1.0, 1.0)

    signo = 1 if codo == "abajo" else -1
    sin_q2 = signo * np.sqrt(1 - cos_q2**2)

    q2 = np.arctan2(sin_q2, cos_q2)
    q1 = np.arctan2(y, x) - np.arctan2(l2 * sin_q2, l1 + l2 * np.cos(q2))

    return q1, q2, q3


def cinematica_directa(q1, q2, q3, l1, l2):
    """
    Calcula la posición cartesiana (x, y, z) a partir de los ángulos de articulación[cite: 3, 4].
    """
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return x, y, q3


def hilo_lectura(ser, detener):
    """Hilo secundario para escuchar respuestas del ESP32 por el puerto serial[cite: 3]."""
    buffer = ""
    while not detener.is_set():
        try:
            if ser and ser.is_open:
                datos = ser.read(ser.in_waiting or 1)
                if datos:
                    buffer += datos.decode("utf-8", errors="replace")
                    while "\n" in buffer:
                        linea, buffer = buffer.split("\n", 1)
                        print(f"[ESP32 RX]: {linea.strip()}")
            else:
                time.sleep(0.1)
        except (serial.SerialException, OSError):
            print("\n[Se perdió la conexión con el puerto serial]")
            break


# =============================================================
# RUTAS PARA SERVIR LA INTERFAZ WEB Y ARCHIVOS ESTÁTICOS
# =============================================================
@app.route('/')
def servir_index():
    return send_from_directory('.', 'index.html')


@app.route('/styles.css')
def servir_css():
    return send_from_directory('.', 'styles.css')


@app.route('/index.js')
def servir_js():
    return send_from_directory('.', 'index.js')


# =============================================================
# ENDPOINTS DE LA API
# =============================================================
@app.route('/api/puertos', methods=['GET'])
def obtener_puertos():
    puertos = list(list_ports.comports())
    lista = [p.device for p in puertos]
    return jsonify({'puertos': lista})


@app.route('/api/conexion', methods=['POST'])
def gestionar_conexion():
    global conexion_serial, hilo_lector, detener_hilo, puerto_actual, baudrate_actual
    global current_q1, current_q2, current_q3

    data = request.json
    accion = data.get('accion')
    puerto = data.get('puerto')
    baudrate = int(data.get('baudrate', 115200))

    if accion == 'conectar':
        if conexion_serial and conexion_serial.is_open:
            return jsonify({'status': 'success', 'message': 'Ya está conectado.'})
        try:
            conexion_serial = serial.Serial()
            conexion_serial.port = puerto
            conexion_serial.baudrate = baudrate
            conexion_serial.timeout = 0.1
            conexion_serial.dtr = False
            conexion_serial.rts = False
            conexion_serial.open()

            puerto_actual = puerto
            baudrate_actual = baudrate

            # Al conectar, se asume que el robot está en la posición inicial (0, 0, 0)
            current_q1 = 0.0
            current_q2 = 0.0
            current_q3 = 0.0

            detener_hilo.clear()
            hilo_lector = threading.Thread(
                target=hilo_lectura, args=(conexion_serial, detener_hilo), daemon=True
            )
            hilo_lector.start()

            return jsonify({
                'status': 'success',
                'message': f'Conectado exitosamente a {puerto}. Posición reseteada a 0.',
            })
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)}), 400

    elif accion == 'desconectar':
        try:
            detener_hilo.set()
            if conexion_serial and conexion_serial.is_open:
                conexion_serial.close()
            return jsonify(
                {'status': 'success', 'message': 'Puerto desconectado correctamente.'}
            )
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)}), 400

    return jsonify({'status': 'error', 'message': 'Acción no válida.'}), 400


@app.route('/api/comando', methods=['POST'])
def procesar_comando():
    global conexion_serial, current_q1, current_q2, current_q3

    data = request.json
    trama_recibida = data.get('trama', '')

    try:
        cuerpo = trama_recibida.strip().lstrip('$').split('*')[0]
        partes = cuerpo.split(',')
        comando = partes[0]

        respuesta_rx = ''

        if comando == 'MOVE_XY':
            x = float(partes[1])
            y = float(partes[2])
            z = 0.0

            # 1. Aplicar Cinemática Inversa para obtener los ángulos objetivo absolutos
            q1_target, q2_target, q3_target = cinematica_inversa(x, y, z, L1, L2, codo='abajo')

            # 2. Calcular el incremento (delta) respecto a la posición actual acumulada
            delta_q1 = np.degrees(q1_target - current_q1)
            delta_q2 = np.degrees(q2_target - current_q2)

            # 3. Actualizar la posición actual del robot
            current_q1 = q1_target
            current_q2 = q2_target
            current_q3 = q3_target

            # Formato requerido por el ESP32: M1,M2,Servo (servo fijo en 0 por ahora)
            comando_esp = f'{delta_q1:.2f},{delta_q2:.2f},0\n'

            if conexion_serial and conexion_serial.is_open:
                conexion_serial.write(comando_esp.encode('ascii'))
                print(f'[Flask TX al ESP32]: {comando_esp.strip()}')
                respuesta_rx = f'Enviado al ESP32 (Delta): {comando_esp.strip()}'
            else:
                return (
                    jsonify({
                        'status': 'error',
                        'message': 'No hay conexión serial activa con el ESP32.',
                    }),
                    400,
                )

        elif comando == 'MOVE_ANG':
            m1_target = float(partes[1])
            m2_target = float(partes[2])

            # Para ángulos directos ingresados por el usuario, calculamos el delta respecto a la posición previa
            current_deg_m1 = np.degrees(current_q1)
            current_deg_m2 = np.degrees(current_q2)

            delta_m1 = m1_target - current_deg_m1
            delta_m2 = m2_target - current_deg_m2

            current_q1 = np.radians(m1_target)
            current_q2 = np.radians(m2_target)

            comando_esp = f'{delta_m1:.2f},{delta_m2:.2f},0\n'

            if conexion_serial and conexion_serial.is_open:
                conexion_serial.write(comando_esp.encode('ascii'))
                respuesta_rx = f'Enviado al ESP32 (Ángulos Delta): {delta_m1:.2f},{delta_m2:.2f},0'
            else:
                return (
                    jsonify({
                        'status': 'error',
                        'message': 'Puerto serial desconectado.',
                    }),
                    400,
                )

        elif comando == 'HOME':
            # Volver a la posición 0 absoluta
            delta_m1 = 0.0 - np.degrees(current_q1)
            delta_m2 = 0.0 - np.degrees(current_q2)
            current_q1 = 0.0
            current_q2 = 0.0
            current_q3 = 0.0

            comando_esp = f'{delta_m1:.2f},{delta_m2:.2f},0\n'
            if conexion_serial and conexion_serial.is_open:
                conexion_serial.write(comando_esp.encode('ascii'))
                respuesta_rx = 'Enviado a HOME (0,0,0)'
            else:
                return jsonify({'status': 'error', 'message': 'Puerto serial desconectado.'}), 400

        else:
            if conexion_serial and conexion_serial.is_open:
                conexion_serial.write(f'{trama_recibida}\n'.encode('ascii'))
                respuesta_rx = f'Enviado comando: {trama_recibida}'

        # Opcional: Ejemplo de verificación usando cinemática directa
        x_dir, y_dir, z_dir = cinematica_directa(current_q1, current_q2, current_q3, L1, L2)
        print(f"[Estado Cinemática Directa Actual] -> X: {x_dir:.2f} cm, Y: {y_dir:.2f} cm")

        return jsonify({
            'status': 'success',
            'message': 'Comando procesado con éxito.',
            'rx': respuesta_rx,
        })

    except ValueError as ve:
        return jsonify({'status': 'error', 'message': f'Error geométrico o de espacio de trabajo: {ve}'}), 400
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 400


if __name__ == '__main__':
    app.run(host='127.0.0.1', port=5000, debug=True)