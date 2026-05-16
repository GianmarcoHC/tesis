# Cable-Driven Parallel Robot (CDPR)
> Sistema de posicionamiento XYZ suspendido por cables para monitoreo y localización mediante visión artificial — Tesis UTEC

---

## ¿Qué es este proyecto?

Un **Cable-Driven Parallel Robot (CDPR)** es un robot paralelo donde los eslabones rígidos son reemplazados por cables. Este proyecto implementa un sistema de posicionamiento 3D que combina control embebido en ESP32 con visión artificial para estimar la posición y orientación del efector final en tiempo real.

---

## Estructura del repositorio

```
tesis/
├── control/        # Firmware ESP32 — motores, encoders y cinemática
└── camara/         # Visión artificial — calibración, AprilTags y pose
```

---

## Módulos

### control/
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

### camara/
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

## Autores

**Gianmarco Humpiri Coila** — UTEC
**Piero Reyes Velarde** — UTEC