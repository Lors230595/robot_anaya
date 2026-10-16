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


# --- Ejemplo de uso ---
if __name__ == "__main__":
    l1, l2 = 10.0, 7.0
    x, y, z = 12.0, 5.0, 3.0

    for codo in ("abajo", "arriba"):
        q1, q2, q3 = cinematica_inversa(x, y, z, l1, l2, codo)
        print(f"Codo {codo}: q1 = {np.degrees(q1):.2f}°, "
              f"q2 = {np.degrees(q2):.2f}°, q3 = {q3:.3f}")
        print("  Verificación (x, y, z):", cinematica_directa(q1, q2, q3, l1, l2))