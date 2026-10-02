# Robot de 3 Grados de Libertad

Este proyecto presenta el desarrollo de un robot manipulador de 3 grados de libertad (3-DOF), diseñado para estudiar el movimiento, la cinemática y el control de un brazo robótico simple.

## Descripción general

El robot cuenta con tres articulaciones que permiten:

- Rotación en la base
- Flexión/levantamiento del brazo
- Movimiento de orientación del efector final

Esto hace que pueda realizar tareas básicas de posicionamiento en un espacio tridimensional, como mover objetos pequeños, realizar pruebas de control y servir como base para futuras ampliaciones.

## Objetivo

El objetivo principal es crear una plataforma didáctica y funcional para:

- Comprender la cinemática directa e inversa
- Explorar el control de motores y actuadores
- Analizar la precisión y el movimiento del efector final
- Servir como base para proyectos de automatización o robótica educativa

## Características

- 3 grados de libertad
- Estructura compacta y ligera
- Diseño modular para fácil mantenimiento
- Compatible con control por microcontrolador o computadora
- Ideal para aprendizaje y demostración

## Arquitectura del robot

El robot se compone de:

1. Base rotatoria
2. Articulación principal
3. Articulación secundaria
4. Efector final o herramienta
5. Sistema de control y potencia

### Esquema conceptual

```text
       Efector final
            |
            |
      Articulación 3
            |
            |
      Articulación 2
            |
            |
      Articulación 1
            |
            |
           Base
```

## Grados de libertad

El robot posee los siguientes movimientos:

- GDL 1: Rotación alrededor del eje vertical (base)
- GDL 2: Elevación del brazo en un plano vertical
- GDL 3: Ajuste de orientación del efector final

## Aplicaciones

- Educación en robótica
- Laboratorios de automatización
- Demostraciones de cinemática robótica
- Prototipos de manipuladores simples
- Desarrollo de algoritmos de control

## Componentes sugeridos

- Servomotores o motores paso a paso
- Estructura metálica o 3D impresa
- Microcontrolador (Arduino, ESP32, Raspberry Pi, etc.)
- Sensores de posición o encoders
- Fuente de alimentación
- Controladores de motor

## Funcionamiento

El sistema puede operar en dos modos principales:

- Control manual: el usuario mueve cada articulación
- Control automático: el robot ejecuta trayectorias o movimientos predefinidos

## Requisitos del proyecto

- Conocimientos básicos de mecánica y robótica
- Comprensión de cinemática directa e inversa
- Familiaridad con programación y control de actuadores
- Herramientas para pruebas y calibración

## Fases del desarrollo

- Diseño mecánico
- Selección de actuadores
- Implementación del sistema de control
- Calibración del robot
- Validación del movimiento y precisión
- Pruebas funcionales

## Ejemplo de uso

```python
# Ejemplo conceptual de movimiento
# Base, brazo y efector final

# Movimiento base
base_angle = 45

# Movimiento del brazo
arm_angle = 30

# Orientación del efector
tool_angle = 20

print("Robot ejecutando trayectoria...")
print(f"Base: {base_angle}°")
print(f"Brazo: {arm_angle}°")
print(f"Efector: {tool_angle}°")
```

## Consideraciones

- Es importante calibrar cada articulación para evitar errores acumulados
- La precisión depende de la rigidez mecánica y del control del sistema
- El uso de sensores mejora la estabilidad y la repetibilidad del movimiento

## Conclusión

El robot de 3 grados de libertad es una excelente base para comprender los principios fundamentales de la robótica industrial y educativa. Su diseño simple permite explorar conceptos clave de cinemática, control y automatización de manera práctica y visual.

## Licencia

Este proyecto puede utilizarse como base para educación, investigación o desarrollo experimental. Se recomienda definir una licencia específica según el uso que se le dé.

## Autor

Proyecto desarrollado como ejemplo de robot manipulador de 3 DOF.
