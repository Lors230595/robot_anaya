import numpy as np

def cinematica_inversa(x, y, z, l1, l2, codo="abajo"):
    """
    Cinemática inversa del robot RRP (Planar RR + Prismatico Z).

    Parámetros:
        x, y, z : Coordenadas del efector final
        l1, l2  : Longitudes de los eslabones
        codo    : "abajo" (q2 > 0) o "arriba" (q2 < 0)

    Retorna:
        (q1, q2, q3) con q1 y q2 en RADIANES y q3 igual a z.
    """
    # Articulación prismática en Z
    q3 = z

    # Distancia al cuadrado en el plano XY
    r2 = x**2 + y**2
    cos_q2 = (r2 - l1**2 - l2**2) / (2 * l1 * l2)

    # Verificar alcanzabilidad geométrica del espacio de trabajo
    if abs(cos_q2) > 1 + 1e-9:
        raise ValueError(
            f"Punto fuera del espacio de trabajo: "
            f"{abs(l1 - l2):.2f} <= sqrt(x²+y²) <= {l1 + l2:.2f}"
        )
    cos_q2 = np.clip(cos_q2, -1.0, 1.0)

    # Selección de solución geométrica (codo arriba / codo abajo)
    signo = 1 if codo == "abajo" else -1
    sin_q2 = signo * np.sqrt(1 - cos_q2**2)

    q2 = np.arctan2(sin_q2, cos_q2)
    q1 = np.arctan2(y, x) - np.arctan2(l2 * sin_q2, l1 + l2 * np.cos(q2))

    return q1, q2, q3


def cinematica_directa(q1, q2, q3, l1, l2):
    """
    Calcula la posición cartesiana (x, y, z) a partir de los ángulos de articulación.
    Nota: q1 y q2 deben estar en RADIANES.
    """
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return x, y, q3