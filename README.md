\# CDPR Thesis Project



Sistema de posicionamiento XYZ suspendido por cables (Cable-Driven Parallel Robot) para monitoreo y localización mediante visión artificial.



\## Estructura del repositorio



```text

.

├── control/

└── camara/

```



\---



\# control/



Código embebido para el ESP32 usando PlatformIO + ESP-IDF.



Funciones principales:

\- Control de motores

\- Lectura de encoders

\- Cinemática del sistema

\- Cálculo de longitudes de cables

\- Control de trayectoria

\- Comunicación serial

\- Pruebas de control



\## Entorno

\- ESP32

\- ESP-IDF

\- PlatformIO



\## Notas

\- El sistema usa FreeRTOS tasks.

\- La lógica principal está en `main/`.

\- Pendiente: implementar control cerrado con feedback visual.



\---



\# camara/



Código relacionado con visión artificial y localización.



Funciones principales:

\- Calibración de cámara

\- Detección de AprilTags

\- Estimación de pose

\- Obtención de posición y orientación

\- Scripts de prueba



\## Librerías

\- OpenCV

\- NumPy

\- AprilTag



\## Notas

\- La calibración se realiza usando tablero checkerboard.

\- Pendiente: integrar feedback en tiempo real con el controlador.



\---



\# Objetivo general



Desarrollar un sistema CDPR capaz de posicionar una plataforma en coordenadas XYZ usando control cinemático y retroalimentación visual.



\---



\# Estado actual



\- \[x] Cinemática básica

\- \[x] Control de motores

\- \[x] Lectura de encoders

\- \[x] Detección de AprilTags

\- \[x] Calibración de cámara

\- \[ ] Integración completa control + visión

\- \[ ] Corrección de error en tiempo real

\- \[ ] Pruebas finales



