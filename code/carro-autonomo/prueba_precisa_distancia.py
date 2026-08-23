import cv2
import numpy as np
import time
import math
import socket
import json
#(np.array([85, 40, 40]), np.array([145, 255, 255])),
# Dirección IP del ESP32
chan = "192.168.137.230"

# Función para enviar los datos vía WiFi al ESP32
def enviar(ip, xd, yd, xo, yo):
    port = 1234
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((ip, port))
    datos = {"x_des": yd,
             "y_des": xd,
             "x_object": yo,
             "y_object": xo}
    json_datos = json.dumps(datos) + "\n"
    client.send(json_datos.encode())
    respuesta = client.recv(1024)
    print("Respuesta del ESP32:", respuesta.decode())
    client.close()

# -------------- Configuración --------------
usar_camara = False  # Cambia a False para usar una imagen en lugar de la cámara
ruta_imagen = r"C:\Users\HP\OneDrive\Escritorio\Rodrigo Gaytan\ITESM\Sexto semestre\Robots\Reto\Circulo_colores_5.png"
radio_real_rojo_cm = 5  # Radio real del objeto rojo en cm

# Función auxiliar para refinar la detección del círculo usando contornos
def refine_circle(mask, x, y, r):
    # Se crea una máscara con el círculo detectado inicialmente
    circle_mask = np.zeros_like(mask)
    cv2.circle(circle_mask, (x, y), r, 255, -1)
    # Se intersecta la máscara original con la máscara del círculo
    mask_circle = cv2.bitwise_and(mask, mask, mask=circle_mask)
    contours, _ = cv2.findContours(mask_circle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # Se toma el contorno con mayor área para refinar la posición
        cnt = max(contours, key=cv2.contourArea)
        (cx, cy), radius = cv2.minEnclosingCircle(cnt)
        return int(cx), int(cy), int(radius)
    else:
        return x, y, r

# -------------- Función de detección de círculos --------------
def detect_circles(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Rangos para cada color (rojo, verde y azul)
    color_ranges = {
        "red": [
            (np.array([0, 90, 90]), np.array([15, 255, 255])),
            (np.array([165, 90, 90]), np.array([180, 255, 255]))
        ],
        "green": [
            (np.array([35, 40, 40]), np.array([85, 255, 255]))
        ],
        "blue": [
            (np.array([90, 50, 70]), np.array([130, 255, 255]))

        ]
    }
    
    # Etiquetas y colores para dibujar
    labels = {"red": "Robot", "green": "Objective", "blue": "Obstacle"}
    color_bgr = {"red": (0, 0, 255), "green": (0, 255, 0), "blue": (255, 0, 0)}
    
    all_circles_vector = []
    radio_referencia_px = None
    origen_x, origen_y = None, None
    
    # Kernel para eliminación de ruido
    kernel = np.ones((7, 7), np.uint8)
    
    # Procesar cada color
    for color, ranges in color_ranges.items():
        mask = np.zeros_like(hsv[:, :, 0])
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, lower, upper)
        
        # Filtros morfológicos para limpiar la máscara
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.GaussianBlur(mask, (9, 9), 2)
    
        # Ajuste de parámetros para mayor precisión en HoughCircles
        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30, 
                                   param1=100, param2=20, minRadius=5, maxRadius=150)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for x, y, r in circles[0, :]:
                # Validación de intensidad: se usa la máscara para confirmar que el círculo tiene suficiente área activa
                circle_mask = np.zeros_like(mask)
                cv2.circle(circle_mask, (x, y), r, 255, -1)
                nonzero = cv2.countNonZero(cv2.bitwise_and(mask, mask, mask=circle_mask))
                area = math.pi * r * r
                if nonzero < 0.5 * area:
                    continue
                
                # Evitar múltiples detecciones para el mismo color
                if any(item[3] == labels[color] for item in all_circles_vector):
                    continue
                
                # Refinar la detección usando contornos
                x_ref, y_ref, r_ref = refine_circle(mask, x, y, r)
                all_circles_vector.append([x_ref, y_ref, r_ref, labels[color]])
                cv2.circle(image, (x_ref, y_ref), r_ref, color_bgr[color], 2)
                # Usamos el círculo rojo (Robot) como referencia
                if color == "red" and radio_referencia_px is None:
                    radio_referencia_px = r_ref
                    origen_x, origen_y = x_ref, y_ref

    # Calcular la escala de conversión de píxeles a metros usando el robot (círculo rojo)
    if radio_referencia_px is not None and radio_referencia_px != 0:
        radio_real_rojo_m = radio_real_rojo_cm * 1.07 / 100.0  # Convertir de cm a m
        escala_px_a_m = radio_real_rojo_m / radio_referencia_px
    else:
        print("No se detectó el círculo rojo de referencia. Usando escala por defecto (1).")
        escala_px_a_m = 1.0

    all_circles_vector_m = []
    for x, y, r, label in all_circles_vector:
        # Coordenadas en metros relativas al centro del robot
        x_rel_px = x - origen_x if origen_x is not None else x
        y_rel_px = origen_y - y if origen_y is not None else y
        x_m = x_rel_px * escala_px_a_m
        y_m = y_rel_px * escala_px_a_m
        radio_m = r * escala_px_a_m
        
        # Distancia y ángulo (en grados) desde el robot
        distancia = math.sqrt(x_m**2 + y_m**2)
        angulo = math.degrees(math.atan2(y_m, x_m))
        
        # Redondeo a dos decimales
        x_m = round(x_m, 2)
        y_m = round(y_m, 2)
        radio_m = round(radio_m, 2)
        distancia = round(distancia, 2)
        angulo = round(angulo, 2)
        
        all_circles_vector_m.append([x_m, y_m, radio_m, label, distancia, angulo])
        
        pos_texto = (x - 30, y - 10)
        cv2.putText(image, f"{label}: ({x_m},{y_m}) d:{distancia}m a:{angulo}°", pos_texto,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    # Dibujo de líneas desde el robot hacia los otros objetos
    if origen_x is not None and origen_y is not None:
        for x_m, y_m, radio_m, label, distancia, angulo in all_circles_vector_m:
            if label in ("Objective", "Obstacle"):
                x_final = int(origen_x + (x_m / escala_px_a_m))
                y_final = int(origen_y - (y_m / escala_px_a_m))
                cv2.line(image, (origen_x, origen_y), (x_final, y_final), (255, 255, 255), 1)
                cv2.putText(image, f"{distancia}m", (x_final, y_final),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return all_circles_vector_m, image

# -------------- Bloque Principal --------------
if usar_camara:
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: No se pudo acceder a la cámara.")
        exit()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error obteniendo frame de la cámara")
            break
        
        # Mostrar el video en vivo
        cv2.imshow("Video en Vivo", frame)
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('p'):
            captured_frame = frame.copy()
            all_circles_vector_m, output = detect_circles(captured_frame)
            datos_filtrados = [dato for dato in all_circles_vector_m if dato[3] in ("Objective", "Obstacle")]
            
            # Mostrar datos detectados
            print("\n--- Datos Detectados ---")
            for dato in datos_filtrados:
                x_m, y_m, radio_m, label, distancia, angulo = dato
                print(f"{label}: Posición = ({x_m} m, {y_m} m), Radio = {radio_m} m, Distancia = {distancia} m, Ángulo = {angulo}°")
            
            # Extraer coordenadas individuales para enviar vía WiFi
            x_des, y_des, x_object, y_object = 0, 0, 0, 0
            for dato in datos_filtrados:
                if dato[3] == "Objective":
                    x_des, y_des = dato[0], dato[1]
                elif dato[3] == "Obstacle":
                    x_object, y_object = dato[0], dato[1]
            print("Enviando datos por WiFi:")
            print("Objective: ({}, {})".format(x_des, y_des))
            print("Obstacle: ({}, {})".format(x_object, y_object))
            enviar(chan, x_des, y_des, x_object, y_object)
            
            cv2.imshow("Imagen Capturada y Procesada", output)
            cv2.waitKey(0)
            
        elif key == ord('q'):
            break
        
        time.sleep(0.1)
    
    cap.release()
    cv2.destroyAllWindows()
else:
    image = cv2.imread(ruta_imagen)
    if image is None:
        print("Error: No se pudo cargar la imagen.")
    else:
        all_circles_vector_m, output = detect_circles(image)
        datos_filtrados = [dato for dato in all_circles_vector_m if dato[3] in ("Objective", "Obstacle")]
        
        print("\n--- Datos Detectados ---")
        for dato in datos_filtrados:
            x_m, y_m, radio_m, label, distancia, angulo = dato
            print(f"{label}: Posición = ({x_m} m, {y_m} m), Radio = {radio_m} m, Distancia = {distancia} m, Ángulo = {angulo}°")
        
        # Extraer coordenadas individuales para enviar vía WiFi
        x_des, y_des, x_object, y_object = 0, 0, 0, 0
        for dato in datos_filtrados:
            if dato[3] == "Objective":
                x_des, y_des = dato[0], dato[1]
            elif dato[3] == "Obstacle":
                x_object, y_object = dato[0], dato[1]
        print("Enviando datos por WiFi:")
        print("Objective: ({}, {})".format(x_des, y_des))
        print("Obstacle: ({}, {})".format(x_object, y_object))
        enviar(chan, x_des, y_des, x_object, y_object)
        
        cv2.imshow("Imagen Procesada", output)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
