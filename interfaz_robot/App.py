import os
import threading
import time
from collections import deque

import numpy as np
import serial
from flask import Flask, jsonify, request, send_from_directory
from serial.tools import list_ports

# Importación de las funciones geométricas
from cinematica_inv import cinematica_inversa, cinematica_directa

# ==============================================================================
# 1. PARÁMETROS CONFIGURABLES DEL ROBOT (MODIFICAR AQUÍ)
# ==============================================================================
L1 = 100.0                      # Longitud del eslabón 1 (cm / mm)
L2 = 100.0                       # Longitud del eslabón 2 (cm / mm)
CODO = "abajo"                 # Configuración geométrica: "abajo" o "arriba"
PASO_JOG = 1.0                 # Desplazamiento manual en cada pulso de D-Pad
Q1_MIN, Q1_MAX = -180.0, 180.0 # Límites mecánicos en grados (Motor 1)
Q2_MIN, Q2_MAX = -150.0, 150.0 # Límites mecánicos en grados (Motor 2)

# ==============================================================================
# 2. CONFIGURACIÓN DEL SERVIDOR Y ESTADO GLOBAL
# ==============================================================================
CARPETA = os.path.dirname(os.path.abspath(__file__))
app = Flask(__name__, static_folder=None)

lock = threading.Lock()
ser = None                     # Objeto serial.Serial activo
detener_lector = threading.Event()
bloqueado = False              # Estado de parada de emergencia

# Posiciones y ángulos absolutos actuales del robot
q_actual = [0.0, 0.0]          # [q1, q2] en grados (Inicia en posición Home: 0°, 0°)
x_actual, y_actual, _ = cinematica_directa(0.0, 0.0, 0.0, L1, L2)

log = deque(maxlen=300)        # Búfer circular para la consola de diagnóstico
log_id = 0


def agregar_log(texto):
    global log_id
    with lock:
        log_id += 1
        log.append((log_id, f"[{time.strftime('%H:%M:%S')}] {texto}"))


# ==============================================================================
# 3. LECTURA SERIAL EN SEGUNDO PLANO (RX)
# ==============================================================================
def hilo_lectura(puerto):
    """Escucha las respuestas del ESP32 e ignora las líneas de telemetría Teleplot ('>')."""
    buffer = ""
    while not detener_lector.is_set():
        try:
            datos = puerto.read(puerto.in_waiting or 1)
        except (serial.SerialException, OSError):
            agregar_log("WARN: Se perdió la conexión serial")
            break
        if not datos:
            continue
        buffer += datos.decode("utf-8", errors="replace")
        while "\n" in buffer:
            linea, buffer = buffer.split("\n", 1)
            linea = linea.strip("\r ")
            if linea and not linea.startswith(">"):
                agregar_log(f"RX: {linea}")


# ==============================================================================
# 4. FUNCIÓN PRINCIPAL DE ENVÍO POR SERIE (SOLO DOS NEMA SEPARADOS POR COMA)
# ==============================================================================
def transmitir_angulos(q1_objetivo, q2_objetivo, es_relativo=True):
    """
    Calcula la trama y transmite únicamente los 2 valores de los motores NEMA.
    Formato hacia la ESP32: "valor1,valor2\n"
    """
    global ser, q_actual

    if ser is None or not ser.is_open:
        raise RuntimeError("No hay conexión con el robot")
    if bloqueado:
        raise RuntimeError("Parada de emergencia activa")

    # Validar límites de seguridad
    if not (Q1_MIN <= q1_objetivo <= Q1_MAX and Q2_MIN <= q2_objetivo <= Q2_MAX):
        raise ValueError(f"Ángulos fuera de rango: q1={q1_objetivo:.1f}°, q2={q2_objetivo:.1f}°")

    if es_relativo:
        # Calcular los deltas de movimiento (d1, d2)
        d1 = q1_objetivo - q_actual[0]
        d2 = q2_objetivo - q_actual[1]
        trama = f"{d1:.3f},{d2:.3f}"
    else:
        # Envío directo de ángulos absolutos
        trama = f"{q1_objetivo:.3f},{q2_objetivo:.3f}"

    with lock:
        # Transmisión por el puerto serie
        ser.write((trama + "\n").encode("ascii"))
        q_actual[0], q_actual[1] = q1_objetivo, q2_objetivo

    agregar_log(f"TX: {trama} (q1={q1_objetivo:.2f}°, q2={q2_objetivo:.2f}°)")
    return trama


# ==============================================================================
# 5. RUTAS DE LA INTERFAZ Y ENDPOINTS API REST
# ==============================================================================
@app.route("/")
def index():
    return send_from_directory(CARPETA, "index.html")

@app.route("/index.js")
def archivo_js():
    return send_from_directory(CARPETA, "index.js")

@app.route("/styles.css")
def archivo_css():
    return send_from_directory(CARPETA, "styles.css")

@app.get("/api/puertos")
def api_puertos():
    """Retorna únicamente los puertos serie físicos detectados en el sistema."""
    return jsonify([p.device for p in list_ports.comports()])

@app.post("/api/conectar")
def api_conectar():
    global ser, bloqueado
    datos = request.get_json() or {}
    puerto = datos.get("puerto")
    baudios = int(datos.get("baudrate", 115200))

    if not puerto:
        return jsonify(error="No se especificó un puerto serial válido"), 400

    if ser and ser.is_open:
        return jsonify(error="Ya existe una conexión abierta"), 400

    nuevo = serial.Serial()
    nuevo.port = puerto
    nuevo.baudrate = baudios
    nuevo.timeout = 0.1
    # Configurar DTR/RTS en False evita reinicios del ESP32
    nuevo.dtr = False
    nuevo.rts = False

    try:
        nuevo.open()
    except serial.SerialException as e:
        return jsonify(error=f"No se pudo abrir {puerto}: {e}"), 500

    ser = nuevo
    bloqueado = False
    detener_lector.clear()
    threading.Thread(target=hilo_lectura, args=(ser,), daemon=True).start()
    agregar_log(f"CONECTADO A {puerto} @ {baudios} BAUDIOS")
    return jsonify(estado())

@app.post("/api/desconectar")
def api_desconectar():
    global ser
    detener_lector.set()
    if ser:
        ser.close()
        ser = None
    agregar_log("DESCONECTADO DEL PUERTO SERIAL")
    return jsonify(estado())

@app.post("/api/mover")
def api_mover():
    """Recibe (X, Y) desde la web, calcula Cinemática Inversa y manda 'd1,d2\\n'."""
    global x_actual, y_actual
    d = request.get_json() or {}
    try:
        x = float(d["x"])
        y = float(d["y"])

        # Cálculo de la Cinemática Inversa
        q1_rad, q2_rad, _ = cinematica_inversa(x, y, 0.0, L1, L2, CODO)
        q1_deg = float(np.degrees(q1_rad))
        q2_deg = float(np.degrees(q2_rad))

        trama = transmitir_angulos(q1_deg, q2_deg, es_relativo=True)
        x_actual, y_actual = x, y

    except (ValueError, RuntimeError) as e:
        agregar_log(f"ERROR: {e}")
        return jsonify(error=str(e)), 400

    return jsonify(trama=trama, q1=q1_deg, q2=q2_deg, **estado())

@app.post("/api/angulos")
def api_angulos():
    """Recibe los ángulos directamente (q1, q2) y envía 'd1,d2\\n'."""
    global x_actual, y_actual
    d = request.get_json() or {}
    try:
        q1 = float(d["q1"])
        q2 = float(d["q2"])

        trama = transmitir_angulos(q1, q2, es_relativo=True)
        x_actual, y_actual, _ = cinematica_directa(np.radians(q1), np.radians(q2), 0.0, L1, L2)

    except (ValueError, RuntimeError) as e:
        agregar_log(f"ERROR: {e}")
        return jsonify(error=str(e)), 400

    return jsonify(trama=trama, q1=q1, q2=q2, **estado())

@app.post("/api/jog")
def api_jog():
    """Mueve el robot un paso discreto en el plano cartesiano X/Y."""
    global x_actual, y_actual
    direccion = request.get_json().get("dir")
    dx, dy = {
        "avanzar": (0, PASO_JOG),
        "retroceder": (0, -PASO_JOG),
        "izquierda": (-PASO_JOG, 0),
        "derecha": (PASO_JOG, 0),
    }.get(direccion, (0, 0))

    try:
        x_nuevo = x_actual + dx
        y_nuevo = y_actual + dy
        q1_rad, q2_rad, _ = cinematica_inversa(x_nuevo, y_nuevo, 0.0, L1, L2, CODO)
        q1_deg = float(np.degrees(q1_rad))
        q2_deg = float(np.degrees(q2_rad))

        trama = transmitir_angulos(q1_deg, q2_deg, es_relativo=True)
        x_actual, y_actual = x_nuevo, y_nuevo

    except (ValueError, RuntimeError) as e:
        agregar_log(f"ERROR: {e}")
        return jsonify(error=str(e)), 400

    return jsonify(trama=trama, q1=q1_deg, q2=q2_deg, **estado())

@app.post("/api/emergencia")
def api_emergencia():
    global bloqueado
    bloqueado = not bloqueado
    agregar_log("PARADA DE EMERGENCIA ACTIVADA" if bloqueado else "Bloqueo liberado")
    return jsonify(estado())

@app.get("/api/estado")
def api_estado():
    return jsonify(estado())

@app.get("/api/log")
def api_log():
    desde = int(request.args.get("desde", 0))
    with lock:
        nuevos = [(i, t) for i, t in log if i > desde]
    return jsonify(nuevos)

def estado():
    return {
        "conectado": bool(ser and ser.is_open),
        "bloqueado": bloqueado,
        "x": x_actual, "y": y_actual,
        "q1": q_actual[0], "q2": q_actual[1],
    }

if __name__ == "__main__":
    app.run(debug=False, threaded=True, host="127.0.0.1", port=5000)