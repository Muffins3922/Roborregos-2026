import cv2
import numpy as np

# 1. Cargamos el diccionario oficial 4x4_50, el mismo que usa el reglamento
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# 2. Generamos cada marcador como imagen (solo la parte interior 4x4, sin el borde negro)
#    marker_size=4 pixeles -> un pixel = una celda, lectura directa sin promediar nada
valores = []
for marker_id in range(50):
    # OpenCV exige generar CON el borde negro incluido (mínimo 6x6: 4x4 interior + 1 celda de borde en cada lado)
    img_completo = np.zeros((6, 6), dtype=np.uint8)
    cv2.aruco.generateImageMarker(dictionary, marker_id, 6, img_completo, 1)

    # Recortamos, quitando la celda de borde de cada lado, para quedarnos solo con la info 4x4
    img = img_completo[1:5, 1:5]

    # 3. Convertimos la cuadrícula de 4x4 píxeles (cada uno 0 o 255) a 16 bits
    bits = (img > 127).astype(int).flatten()  # 1 = blanco, 0 = negro
    valor_binario = 0
    for b in bits:
        valor_binario = (valor_binario << 1) | int(b)
    valores.append(valor_binario)

# 4. Imprimimos en formato listo para pegar en C++
print("const uint16_t codigosArUco[50] = {")
for i, v in enumerate(valores):
    print(f"  0b{v:016b},  // ID {i}")
print("};")