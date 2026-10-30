import numpy as np

# Funciones trigonométricas en grados
def sind(a):
    return np.sin(np.radians(a))

def cosd(a):
    return np.cos(np.radians(a))

def atan2d(y, x):
    return np.degrees(np.arctan2(y, x))


def cinematica_directa(q1, q2, q3, l1, l2):
    """
    Cinemática directa del robot RRP.

    Parámetros:
        q1, q2 : ángulos en GRADOS
        q3     : desplazamiento prismático
        l1, l2 : longitudes de los eslabones

    Retorna:
        x, y, z
    """
    x = l1 * cosd(q1) + l2 * cosd(q1 + q2)
    y = l1 * sind(q1) + l2 * sind(q1 + q2)
    z = q3

    return x, y, z


def cinematica_inversa(x, y, z, l1, l2, codo=1):
    """
    Cinemática inversa del robot RRP.

    Parámetros:
        x, y, z : posición deseada del efector
        l1, l2  : longitudes de los eslabones
        codo    : +1 (q2 > 0) o -1 (q2 < 0)

    Retorna:
        q1, q2 en GRADOS, q3 = z
    """
    cos_q2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    if not -1 <= cos_q2 <= 1:
        raise ValueError("La posición no es alcanzable con esas longitudes.")

    sin_q2 = codo * np.sqrt(1 - cos_q2**2)

    q2 = atan2d(sin_q2, cos_q2)
    q1 = atan2d(y, x) - atan2d(l2 * sin_q2, l1 + l2 * cos_q2)
    q3 = z

    return q1, q2, q3


if __name__ == "__main__":
    print("Cinemática inversa del robot RRP")
    l1 = float(input("Ingrese la longitud del eslabón l1: "))
    l2 = float(input("Ingrese la longitud del eslabón l2: "))
    x = float(input("Ingrese x: "))
    y = float(input("Ingrese y: "))
    z = float(input("Ingrese z: "))
    codo = int(input("Ingrese la configuración del codo (1 o -1): "))

    try:
        q1, q2, q3 = cinematica_inversa(x, y, z, l1, l2, codo)
        print(f"q1 = {q1:.2f} grados")
        print(f"q2 = {q2:.2f} grados")
        print(f"q3 = {q3:.2f}")
    except ValueError as error:
        print(f"Error: {error}")