# Carro Autónomo — CINVESTAV

Plataforma móvil autónoma que navega hacia un objetivo evitando un obstáculo, usando visión
computacional para localización y un ESP32 para el control de bajo nivel de los motores.

## Cómo funciona

1. **`prueba_precisa_distancia.py`** (Python + OpenCV) — corre en una laptop conectada a una cámara.
   Detecta tres círculos de color en la imagen (rojo = robot, verde = objetivo, azul = obstáculo),
   y usa el tamaño conocido del círculo rojo como referencia para convertir posiciones de píxeles a
   metros. Envía las coordenadas del objetivo y el obstáculo al ESP32 por WiFi (socket TCP, mensaje
   JSON).

2. **`Palomo_Puro.ino`** (C++ / ESP32, Arduino framework) — recibe las coordenadas por WiFi y calcula
   la trayectoria con un algoritmo de campos potenciales artificiales (atracción hacia el objetivo,
   repulsión del obstáculo). Simula la cinemática del robot (integración Runge-Kutta de 4º orden)
   para obtener velocidades de rueda objetivo, y las alcanza con un control PI sobre las RPM leídas
   de los encoders de cada motor.

## Requisitos

- ESP32 con las librerías `WiFi.h` y `ArduinoJson`.
- Python 3 con `opencv-python` y `numpy`.
- Ambos dispositivos en la misma red WiFi local.

## Notas

- Las credenciales de WiFi en `Palomo_Puro.ino` están redactadas (`YOUR_WIFI_SSID` /
  `YOUR_WIFI_PASSWORD`) — reemplázalas por las tuyas antes de compilar.
- La IP del ESP32 y la ruta de imagen de prueba en `prueba_precisa_distancia.py` están fijas para el
  entorno original de desarrollo; ajústalas a tu configuración local.

Este código se comparte como evidencia técnica del proyecto, no como un paquete listo para
ejecutarse "out of the box".
