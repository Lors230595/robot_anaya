#!/usr/bin/env python3
"""
Envía por el puerto serial dos números flotantes separados por coma y un Enter,
por ejemplo:  100,150   o   -45.5,90

Requisitos:  pip install pyserial
Uso:         python enviar_motores.py                (elige el puerto de una lista)
             python enviar_motores.py -p COM5        (Windows)
             python enviar_motores.py -p /dev/ttyUSB0
             python enviar_motores.py --teleplot     (muestra también las líneas ">M1_..." de Teleplot)
"""

import argparse
import sys
import threading

import serial
from serial.tools import list_ports
import numpy as np

def cinematica_inversa(x, y, z, l1, l2, codo="abajo"):
    """
    Cinemática inversa del robot RRP (DH: [q1 0 l1 0; q2 0 l2 0; 0 q3 0 0]).

    Parámetros:
        x, y, z : coordenadas del efector final
        l1, l2  : longitudes de los eslabones
        codo    : "abajo" (sin q2 > 0) o "arriba" (sin q2 < 0)

    Retorna:
        (q1, q2, q3) con q1 y q2 en radianes y q3 en las unidades de z.
    """
    # Articulación prismática
    q3 = z

    # Ley de cosenos
    r2 = x**2 + y**2
    cos_q2 = (r2 - l1**2 - l2**2) / (2 * l1 * l2)

    # Verificar alcanzabilidad (tolerancia numérica)
    if abs(cos_q2) > 1 + 1e-9:
        raise ValueError(
            f"Punto fuera del espacio de trabajo: "
            f"{abs(l1 - l2):.3f} <= sqrt(x²+y²) <= {l1 + l2:.3f}"
        )
    cos_q2 = np.clip(cos_q2, -1.0, 1.0)

    # Selección de codo
    signo = 1 if codo == "abajo" else -1
    sin_q2 = signo * np.sqrt(1 - cos_q2**2)

    q2 = np.arctan2(sin_q2, cos_q2)
    q1 = np.arctan2(y, x) - np.arctan2(l2 * sin_q2, l1 + l2 * np.cos(q2))

    return q1, q2, q3


def cinematica_directa(q1, q2, q3, l1, l2):
    """Para verificar la solución."""
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return x, y, q3


def elegir_puerto():
    puertos = list(list_ports.comports())
    if not puertos:
        sys.exit("No se encontraron puertos seriales. Conecta el ESP32 y vuelve a intentar.")
    if len(puertos) == 1:
        print(f"Usando puerto: {puertos[0].device} ({puertos[0].description})")
        return puertos[0].device

    print("Puertos disponibles:")
    for i, p in enumerate(puertos):
        print(f"  [{i}] {p.device} - {p.description}")
    while True:
        eleccion = input("Elige el numero del puerto: ").strip()
        if eleccion.isdigit() and int(eleccion) < len(puertos):
            return puertos[int(eleccion)].device
        print("  Opcion invalida.")


def hilo_lectura(ser, detener, mostrar_teleplot):
    """Imprime lo que responde el ESP32 (sin las líneas de Teleplot, salvo que se pida)."""
    buffer = ""
    while not detener.is_set():
        try:
            datos = ser.read(ser.in_waiting or 1)
        except (serial.SerialException, OSError):
            print("\n[Se perdio la conexion con el puerto serial]")
            detener.set()
            break
        if not datos:
            continue

        buffer += datos.decode("utf-8", errors="replace")

        if mostrar_teleplot:
            sys.stdout.write(buffer)
            sys.stdout.flush()
            buffer = ""
            continue

        # Filtrar líneas de Teleplot (empiezan con '>')
        while "\n" in buffer:
            linea, buffer = buffer.split("\n", 1)
            if not linea.startswith(">"):
                sys.stdout.write(linea.rstrip("\r") + "\n")
        # Texto parcial (por ejemplo el prompt, que no termina en salto de línea)
        if buffer and not buffer.startswith(">"):
            sys.stdout.write(buffer)
            buffer = ""
        sys.stdout.flush()


def parsear(texto):
    """Devuelve (a, b) si el texto es 'numero,numero'; si no, None."""
    partes = texto.replace(" ", "").split(",")
    if len(partes) != 2:
        return None
    try:
        return float(partes[0]), float(partes[1])
    except ValueError:
        return None


def main():

    l1, l2 = 10.0, 7.0
    x, y, z = 12.0, 5.0, 3.0
    parser = argparse.ArgumentParser(description="Enviar 'M1,M2' al ESP32 por serial")
    parser.add_argument("-p", "--puerto", help="Puerto serial (ej: COM5 o /dev/ttyUSB0)")
    parser.add_argument("-b", "--baudios", type=int, default=115200, help="Velocidad (por defecto 115200)")
    parser.add_argument("--teleplot", action="store_true", help="Mostrar tambien las lineas de Teleplot")
    args = parser.parse_args()

    puerto = args.puerto or elegir_puerto()

    # Se configura DTR/RTS en False ANTES de abrir para evitar (en lo posible) que el ESP32 se reinicie
    ser = serial.Serial()
    ser.port = puerto
    ser.baudrate = args.baudios
    ser.timeout = 0.1
    ser.dtr = False
    ser.rts = False
    try:
        ser.open()
    except serial.SerialException as e:
        sys.exit(f"No se pudo abrir {puerto}: {e}")

    print(f"Conectado a {puerto} @ {args.baudios} baudios.")
    print("Escribe dos numeros separados por coma (ej: 100,150) y Enter. 'q' para salir.\n")

    detener = threading.Event()
    lector = threading.Thread(target=hilo_lectura, args=(ser, detener, args.teleplot), daemon=True)
    lector.start()

    try:
        while not detener.is_set():
            try:
                texto = input().strip()
            except EOFError:
                break

            if texto.lower() in ("q", "salir", "exit"):
                break
            if not texto:
                continue

            valores = parsear(texto)
            if valores is None:
                print("  Formato invalido. Use: numero,numero (ej: 100,150 o -45.5,90)")
                continue
            q1, q2, q3 = cinematica_inversa(x, y, z, l1, l2, "abajo")
            valores = (np.degrees(q1), np.degrees(q2))
            ser.write(f"{valores[0]},{valores[1]}\n".encode("ascii"))
    except KeyboardInterrupt:
        pass
    finally:
        detener.set()
        lector.join(timeout=1)
        ser.close()
        print("\nPuerto cerrado.")


if __name__ == "__main__":
    main()