# Cable-Driven Parallel Robot (CDPR)

> Sistema de posicionamiento XYZ suspendido por cables para monitoreo y localización mediante visión artificial — Tesis UTEC

---

## ¿Qué es este proyecto?

Un **Cable-Driven Parallel Robot (CDPR)** es un robot paralelo donde los eslabones rígidos son reemplazados por cables. Este proyecto implementa un sistema de posicionamiento 3D que combina control embebido en ESP32 con visión artificial para estimar la posición y orientación del efector final en tiempo real.

El repositorio incluye tanto la implementación embebida (firmware, visión) como el modelado y análisis en MATLAB/Simulink utilizados durante el diseño y la sintonización del sistema de control.

---

## Estructura del repositorio

```
tesis/
├── cad/                    # Modelos 3D del prototipo
├── camara/                 # Visión artificial — calibración, AprilTags y pose
├── control/                # Firmware ESP32 — motores, encoders y cinemática
└── matlab/                 # Modelado, control y simulación en MATLAB/Simulink
    ├── implementacion/     # Identificación del motor, sintonización PID (AG)
    │                       # y control cinemático (cámara → Simulink → ESP32)
    └── simulacion/         # Simulación completa con Simscape Multibody
```

---

## Módulos

### `cad/`

Modelos 3D del sistema mecánico del prototipo.

**Formato:** Autodesk Inventor / STL

---

### `camara/`

Pipeline de visión artificial para localización del robot.

| Componente | Descripción |
|---|---|
| Calibración | Estimación de parámetros intrínsecos de cámara |
| AprilTags | Detección de marcadores fiduciales |
| Pose | Estimación de posición y orientación 6DOF |
| Scripts | Herramientas de prueba y validación |

**Dependencias:**
- Python 3.x
- OpenCV
- NumPy
- AprilTag

---

### `control/`

Firmware embebido para el **ESP32** desarrollado con PlatformIO + ESP-IDF.

| Componente | Descripción |
|---|---|
| Motores | Control de velocidad y dirección |
| Encoders | Lectura de posición angular |
| Cinemática | Cálculo de longitudes de cable |
| Trayectoria | Planificación y seguimiento de trayectorias |
| Comunicación | Interfaz serial con PC |

**Stack:**
- ESP32 + FreeRTOS
- ESP-IDF
- PlatformIO

> La lógica principal vive en `main/`. El sistema corre tareas concurrentes con FreeRTOS.

---

### `matlab/`

Modelado, sintonización y simulación del sistema de control, dividido en dos subcarpetas según su propósito.

#### `matlab/implementacion/`

Scripts y modelo empleados para llevar el controlador desde el diseño hasta el prototipo físico: identifican la planta real, optimizan las ganancias del PID y ejecutan el lazo de control cinemático que corre en conjunto con el prototipo.

| Componente | Descripción |
|---|---|
| Identificación del motor | Estimación de los parámetros del modelo de primer orden del motor DC (ganancia K y constante de tiempo τ) a partir de datos experimentales |
| Sintonización PID (AG) | Optimización de las ganancias Kp, Ki, Kd del lazo interno mediante algoritmo genético |
| Control cinemático (Simulink) | Modelo que recibe la posición y orientación estimadas por la cámara, calcula el error respecto a la trayectoria de referencia y genera las señales de control enviadas al ESP32 |

**Stack:** MATLAB, Simulink

#### `matlab/simulacion/`

Modelo de simulación del sistema completo, independiente del hardware, usado para validar el comportamiento dinámico del efector antes de la implementación.

| Componente | Descripción |
|---|---|
| Modelo multicuerpo | Simulación de la dinámica del efector final suspendido por cables, desarrollada con Simscape Multibody |

**Stack:** MATLAB, Simulink, Simscape Multibody

---

## Autores

**Gianmarco Humpiri Coila** — UTEC
**Piero Reyes Velarde** — UTEC
