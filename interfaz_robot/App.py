from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import math
import time
import serial
import serial.tools.list_ports

app = Flask(__name__, static_url_path='', static_folder='.')
CORS(app)

# Longitudes físicas del Robot RRP (cm)
L1 = 10.0
L2 = 7.0

# Posición Inicial / Estado Actual en Memoria (Inicia en HOME)
posicion_actual = {
    "x": 10.0,
    "y": -7.0,
    "q1": 0.0,
    "q2": -90.0,
    "z": 0.0
}

conexion_serial = {
    "objeto_serial": None,
    "conectado": False,
    "puerto": None,
    "baudrate": 115200
}

@app.route('/')
def servidor_html():
    return send_from_directory('.', 'index.html')

def atan2d(y, x):
    return math.degrees(math.atan2(y, x))

def cinematica_inversa(x, y, z=0.0, codo=-1):
    """Calcula la cinemática inversa sin restricción de ángulos de los motores"""
    cos_q2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if not -1.0 <= cos_q2 <= 1.0:
        raise ValueError("Posición fuera del alcance físico de las longitudes L1 y L2.")

    sin_q2 = codo * math.sqrt(max(0.0, 1.0 - cos_q2**2))

    q2 = atan2d(sin_q2, cos_q2)
    q1 = atan2d(y, x) - atan2d(L2 * sin_q2, L1 + L2 * cos_q2)
    q3 = z

    return round(q1, 2), round(q2, 2), round(q3, 2)

def calcular_checksum(cadena):
    checksum = 0
    for char in cadena:
        checksum ^= ord(char)
    return f"{checksum:02X}"

@app.route('/api/puertos', methods=['GET'])
def obtener_puertos():
    puertos_disponibles = []
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        puertos_disponibles.append({
            "puerto": port.device,
            "label": f"{port.device} - {port.description}"
        })

    if not puertos_disponibles:
        puertos_disponibles.append({
            "puerto": "COM_PRUEBA",
            "label": "Sin puertos detectados (Modo Simulación)"
        })

    return jsonify({"puertos": puertos_disponibles}), 200

@app.route('/api/conectar', methods=['POST'])
def conectar():
    data = request.json or {}
    puerto = data.get("puerto")
    baudrate = int(data.get("baudrate", 115200))

    if not puerto:
        return jsonify({"status": "error", "mensaje": "Selecciona un puerto válido"}), 400

    if conexion_serial["objeto_serial"] and conexion_serial["objeto_serial"].is_open:
        conexion_serial["objeto_serial"].close()

    try:
        if puerto != "COM_PRUEBA":
            ser = serial.Serial(puerto, baudrate, timeout=1)
            conexion_serial["objeto_serial"] = ser
            conexion_serial["conectado"] = True
            conexion_serial["puerto"] = puerto
            conexion_serial["baudrate"] = baudrate
            msg = f"Conexión Serial exitosa con ESP en {puerto} a {baudrate} baudios."
        else:
            conexion_serial["conectado"] = True
            conexion_serial["puerto"] = puerto
            conexion_serial["baudrate"] = baudrate
            msg = "ESP conectada en modo simulación (sin puerto físico)."

        return jsonify({"status": "ok", "mensaje": msg}), 200

    except Exception as e:
        conexion_serial["conectado"] = False
        return jsonify({"status": "error", "mensaje": f"No se pudo abrir {puerto}: {str(e)}"}), 500

@app.route('/api/comando', methods=['POST'])
def procesar_comando():
    if not conexion_serial["conectado"]:
        return jsonify({"status": "error", "mensaje": "Primero debes conectar con la ESP"}), 400

    datos = request.json or {}
    tipo = datos.get("tipo")

    # MODO 1: CRUCETA D-PAD (Pasos relativas de 0.25 cm / 2.5 mm sin HOME previo)
    if tipo in ["STEP_X", "STEP_Y"]:
        dx = datos.get("dx", 0.0)
        dy = datos.get("dy", 0.0)

        x_nueva = round(posicion_actual["x"] + dx, 2)
        y_nueva = round(posicion_actual["y"] + dy, 2)

        try:
            q1, q2, q3 = cinematica_inversa(x_nueva, y_nueva, posicion_actual["z"])
            
            posicion_actual["x"] = x_nueva
            posicion_actual["y"] = y_nueva
            posicion_actual["q1"] = q1
            posicion_actual["q2"] = q2

            contenido = f"STEP,H0,X:{x_nueva},Y:{y_nueva},Q1:{q1},Q2:{q2},Z:{posicion_actual['z']}"
            checksum = calcular_checksum(contenido)
            trama = f"<{contenido}|CS:{checksum}>"

            log_msg = f"[{time.strftime('%H:%M:%S')}] Paso manual (0.25 cm) -> Posición: (X:{x_nueva}, Y:{y_nueva}) | Q1:{q1}°, Q2:{q2}°"

            return jsonify({
                "status": "ok",
                "trama": trama,
                "checksum": f"0x{checksum} (VÁLIDO)",
                "log": log_msg,
                "posicion": posicion_actual
            }), 200

        except ValueError as err:
            return jsonify({"status": "error", "mensaje": f"Movimiento no alcanzable: {str(err)}"}), 400

    # MODO 2: MOVER A COORDENADAS (X, Y) - Con HOME previo
    elif tipo == "MOVE_XY":
        x_target = datos.get("x", 0.0)
        y_target = datos.get("y", 0.0)

        try:
            q1, q2, q3 = cinematica_inversa(x_target, y_target, posicion_actual["z"])

            posicion_actual["x"] = x_target
            posicion_actual["y"] = y_target
            posicion_actual["q1"] = q1
            posicion_actual["q2"] = q2

            contenido = f"MOVE_XY,H1,X:{x_target},Y:{y_target},Q1:{q1},Q2:{q2},Z:{posicion_actual['z']}"
            checksum = calcular_checksum(contenido)
            trama = f"<{contenido}|CS:{checksum}>"

            log_msg = f"[{time.strftime('%H:%M:%S')}] Secuencia: 1. HOME -> 2. Ir a (X:{x_target}, Y:{y_target}) | Q1:{q1}°, Q2:{q2}°"

            return jsonify({
                "status": "ok",
                "trama": trama,
                "checksum": f"0x{checksum} (VÁLIDO)",
                "log": log_msg,
                "posicion": posicion_actual
            }), 200

        except ValueError as err:
            return jsonify({"status": "error", "mensaje": str(err)}), 400

    # MODO 3: MOVER ÁNGULOS (Q1, Q2) - Con HOME previo
    elif tipo == "MOVE_ANG":
        q1_target = datos.get("q1", 0.0)
        q2_target = datos.get("q2", 0.0)

        x_calc = round(L1 * math.cos(math.radians(q1_target)) + L2 * math.cos(math.radians(q1_target + q2_target)), 2)
        y_calc = round(L1 * math.sin(math.radians(q1_target)) + L2 * math.sin(math.radians(q1_target + q2_target)), 2)

        posicion_actual["x"] = x_calc
        posicion_actual["y"] = y_calc
        posicion_actual["q1"] = q1_target
        posicion_actual["q2"] = q2_target

        contenido = f"MOVE_ANG,H1,X:{x_calc},Y:{y_calc},Q1:{q1_target},Q2:{q2_target},Z:{posicion_actual['z']}"
        checksum = calcular_checksum(contenido)
        trama = f"<{contenido}|CS:{checksum}>"

        log_msg = f"[{time.strftime('%H:%M:%S')}] Secuencia: 1. HOME -> 2. Mover Ángulos Q1:{q1_target}°, Q2:{q2_target}° | Resultante (X:{x_calc}, Y:{y_calc})"

        return jsonify({
            "status": "ok",
            "trama": trama,
            "checksum": f"0x{checksum} (VÁLIDO)",
            "log": log_msg,
            "posicion": posicion_actual
        }), 200

    # OTROS COMANDOS
    elif tipo == "HOME":
        posicion_actual["x"] = 10.0
        posicion_actual["y"] = -7.0
        posicion_actual["q1"] = 0.0
        posicion_actual["q2"] = -90.0
        posicion_actual["z"] = 0.0

        contenido = "HOME,H1,X:10.0,Y:-7.0,Q1:0.0,Q2:-90.0,Z:0.0"
        checksum = calcular_checksum(contenido)
        trama = f"<{contenido}|CS:{checksum}>"

        log_msg = f"[{time.strftime('%H:%M:%S')}] Rutina HOME ejecutada -> Origen: Q1:0°, Q2:-90°"

        return jsonify({
            "status": "ok",
            "trama": trama,
            "checksum": f"0x{checksum} (VÁLIDO)",
            "log": log_msg,
            "posicion": posicion_actual
        }), 200

    elif tipo == "SERVO_Z":
        z_val = datos.get("z", 0.0)
        posicion_actual["z"] = z_val

        contenido = f"SERVO_Z,H0,X:{posicion_actual['x']},Y:{posicion_actual['y']},Q1:{posicion_actual['q1']},Q2:{posicion_actual['q2']},Z:{z_val}"
        checksum = calcular_checksum(contenido)
        trama = f"<{contenido}|CS:{checksum}>"

        estado_z = "Arriba (0)" if z_val == 0 else "Abajo (1)"
        log_msg = f"[{time.strftime('%H:%M:%S')}] Servo Z cambiado a: {estado_z}"

        return jsonify({
            "status": "ok",
            "trama": trama,
            "checksum": f"0x{checksum} (VÁLIDO)",
            "log": log_msg,
            "posicion": posicion_actual
        }), 200

    elif tipo == "STOP":
        contenido = f"STOP,H0,X:{posicion_actual['x']},Y:{posicion_actual['y']},Q1:{posicion_actual['q1']},Q2:{posicion_actual['q2']},Z:{posicion_actual['z']}"
        checksum = calcular_checksum(contenido)
        trama = f"<{contenido}|CS:{checksum}>"

        log_msg = f"[{time.strftime('%H:%M:%S')}] ¡PARADA DE EMERGENCIA ACTIVADA!"

        return jsonify({
            "status": "ok",
            "trama": trama,
            "checksum": f"0x{checksum} (VÁLIDO)",
            "log": log_msg,
            "posicion": posicion_actual
        }), 200

    return jsonify({"status": "error", "mensaje": "Comando no reconocido"}), 400

if __name__ == '__main__':
    app.run(host='127.0.0.1', port=5000, debug=True)