# Robot de Eslabón Flexible — CINVESTAV

Sistema de control para una viga flexible de un solo eslabón, donde un agente de aprendizaje por
refuerzo (RL) ajusta de forma autónoma las ganancias de un controlador PID, logrando una mejora del
11% en la respuesta del sistema y suprimiendo vibraciones estructurales.

## Cómo funciona

1. **`oneBar.cpp`** (C++) — control en tiempo real que corre sobre una tarjeta de adquisición de
   datos Sensoray 826. Lee el ángulo de la viga por encoder y su aceleración por acelerómetro,
   calcula el error contra una referencia, y aplica el PID a través de salidas PWM que mueven el
   motor. Registra cada corrida en `data.csv` para análisis posterior.

2. **`oneCamera.cpp`** (C++) — proceso independiente que usa una cámara Intel RealSense + OpenCV
   para rastrear la posición de la punta de la viga en tiempo real, y la comparte con `oneBar.cpp`
   a través de memoria compartida (POSIX shared memory).

3. **`RICL (2).ipynb`** (Python, Google Colab) — con los datos capturados por `oneBar.cpp`:
   - Ajusta un modelo lineal de espacio de estados de la dinámica de la viga (regresión Ridge).
   - Entrena un agente de aprendizaje por refuerzo Soft Actor-Critic (SAC, vía
     `stable-baselines3`) que explora distintas combinaciones de ganancias Kp/Ki/Kd sobre ese
     modelo aprendido, hasta encontrar el ajuste que minimiza el error de seguimiento y las
     vibraciones — el origen de la mejora del 11% mencionada en el sitio.

## Requisitos

- `oneBar.cpp` / `oneCamera.cpp`: SDK de Sensoray 826, Intel RealSense SDK (librealsense2), OpenCV,
  Eigen3 y ZeroMQ — dependen de hardware específico de laboratorio y no corren en una máquina
  cualquiera.
- `RICL (2).ipynb`: Python 3 con `pandas`, `scikit-learn`, `gymnasium` y `stable-baselines3`
  (se instala dentro del propio notebook).

## Notas

Este código se comparte como evidencia técnica del proyecto — la parte de control en tiempo real
depende de hardware de laboratorio específico (DAQ + cámara RealSense) y no está pensada para
ejecutarse fuera de ese entorno. El notebook de RL sí puede correrse de forma independiente si se
cuenta con un archivo de datos (`data.csv`) con el mismo formato.
