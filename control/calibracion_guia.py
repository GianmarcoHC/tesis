
import cv2 as cv
import numpy as np
import glob # Para importar los nombres de las imágenes
import matplotlib.pyplot as plt   # Para mostrar las imágenes

#################################################
### ---- 1. Puntos en el objeto (índices) --- ###
#################################################
# --- CONFIGURA según tu tablero ---
# Para la calibración en OpenCV son importantes solamente los puntos que determinan las intersecciones internas; es decir, aquellos puntos rodeados por 
# dos cuadrados negros y dos cuadrados blancos. Se puede determinar que hay 7 intersecciones internas en la horizontal (eje X), 
# y 5 puntos en la vertical (eje Y), dando un total de 35 intersecciones. A estos puntos se les denomina "puntos del objeto" ("object points") y 
# cada uno se representará como (Xi,Yi,0), donde Xi Yi son enteros que comienzan en cero. Así, a la primera intersección se le asociará el punto 
# (0,0,0), a la segunda (1,0,0), y así hasta llegar al último punto (6,4,0).
pattern_size = (7, 5)   # ESQUINAS INTERNAS (con 8x6 cuadros → 7x5 esquinas)
#square_size  = 0.025    # cada cuadrito 25 mm → 0.025 m
# ----------------------------------


# Puntos 3D del patrón (Z=0)
# Preparar los puntos del objeto: (0,0,0), (1,0,0), (2,0,0) ..., (6,4,0)
objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(pattern_size[0]*pattern_size[1], 2)
#objp *= square_size
print("Primeros puntos:"); print(objp[0:4])
print("Últimos puntos:"); print(objp[-4:])

#################################################
### ---- 2. Puntos en la imagen (corners) --- ###
#################################################


images = sorted(glob.glob("calib_camara/*.jpg"))   # busca todas las fotos
print("Número de imágenes:", len(images))
print("Ejemplo de imagen:", images[0])
imsize = None


# Lectura de una imagen
I = cv.imread(images[0])

# Tamaño de la imagen
I_size = (I.shape[1], I.shape[0])
print(f"Tamaño de la imagen: (x,y)=({I_size[0]}, {I_size[1]})")

# Criterio de término: epslon y máximo de iteraciones
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 1e-4)

# Conversión de la imagen a escala de grises
Igray = cv.cvtColor(I, cv.COLOR_BGR2GRAY)

# Encontrar las esquinas (corners) denominadas "puntos de la imagen" ("image points")
retval, corners = cv.findChessboardCorners(Igray, pattern_size, None)

# Si se encuentra los puntos (corners), mostrar la imagen con los puntos
if retval == True:
    # Añadir precisión de sub-pixel
    corners2 = cv.cornerSubPix(Igray, corners, (11,11), (-1,-1), criteria)
    # Añadir los puntos (corners) encontrados a la imagen
    cv.drawChessboardCorners(I, pattern_size, corners2, retval)
    # Mostrar la imagen
    plt.figure(figsize=(8,8))
    plt.imshow(I, cmap='gray')
    plt.axis('off'); plt.show()
    
# Notar el orden en el que se encuentran los puntos: de izquierda a derecha y de arriba hacia abajo. E
# Este orden permite prever con antelación las coordenadas de estos puntos en 3D conociendo las dimensiones de cada cuadrado.


# A continuación se aplicará esta misma detección de esquinas pero ahora a todas las imágenes del tablero, y se agregará las 
# coordenadas de las esquinas (en píxeles) a la lista 'img_points'.  Igualmente, se agregará las coordenadas (índices) de los 
# puntos a la lista 'obj_points'. Debido a que se conoce el orden de los puntos, se puede prever su correspondencia en 3D 
# (es decir, la correspondencia entre cada elemento de 'img_points' con los índices 'obj_points'.

# Almacén de puntos del objeto (índices) y puntos de la imagen (corners)
obj_points = []     # Puntos de cada objeto (3d)
img_points = []     # Puntos de cada imagen (2d)
show_images = False # Mostrar las imágenes?

# Bucle para todas las imágenes, buscando las "esquinas" (corners)
for idx, iname in enumerate(images):
    # Leer la imagen
    I = cv.imread(iname)
    # Convertir a escala de grises
    Igray = cv.cvtColor(I, cv.COLOR_BGR2GRAY)

    # Encontrar las esquinas (corners) internas del patrón (de cuadrados negros y blancos)
    retval, corners = cv.findChessboardCorners(Igray, pattern_size, None)

    # Si se encuentra los puntos, añadirlos a la lista
    if retval == True:
        # Incrementar la exactitud de los puntos
        corners2 = cv.cornerSubPix(Igray, corners, (11,11), (-1,-1), criteria)
        # Puntos del objeto y puntos de la imagen, para la imagen actual
        obj_points.append(objp)
        img_points.append(corners2)

        # Añadir a la imagen los puntos (corners) encontrados
        cv.drawChessboardCorners(I, pattern_size, corners, retval)

        # Almacenar las imágenes con sus esquinas ("corners")
        # cv2.imwrite('corners'+str(idx)+'.jpg', I)

        # Mostrar la imagen
        if (show_images):
            plt.imshow(I, cmap='gray')
            plt.axis('off'); plt.show()
            

#################################################
### ---- 3. Calibración de la Cámara --- ###
#################################################    
 
# La calibración de la cámara se realiza usando la función 'calibrateCamera', teniendo como entrada los puntos 
# del objeto 'obj_points', los puntos correspondientes en la 'imagen img_points', y el tamaño de las imágenes.


# Calibración de la cámara dados los puntos objeto (3D) y los puntos imágen (corners) correspondientes
retval, M, coefs_dist, rvecs, tvecs = cv.calibrateCamera(obj_points, img_points, I_size, None, None)


# Mostrar los los parámetros
print("Matriz de calibración:"); print(np.round(M,3))
print("\nCoeficientes de distorsión:"); print(np.round(coefs_dist,3))
print("Error de reproyección promedio (RMS):", retval)

# Los parámetros pueden ser almacenados en un archivo binario, usando la biblioteca llamada pickle, 
# para luego ser cargados cuando sea necesario. Los parámetros calibrados se almacenarán en formato de diccionario 
# (pero en binario) en el archivo camera_params.p.

# Grabar los valores usando pickle
import pickle

# Diccionario para almacenar parámetros calibrados
params = {}
params["M"] = M
params["coefs_dist"] = coefs_dist

# Almacenar los parámetros de la cámara
pickle.dump(params, open("camera_params_2.npz", "wb")) #cambiar
print("Parámetros de la cámara: GUARDADO")


#################################################
### ---- 4. Aplicación de la Calibración a una Imagen --- ###
#################################################    

# Los parámetros intrínsecos obtenidos durante la calibración de la cámara pueden ser obtenidos para quitar 
# la distorsión a una imagen. Con este fin se utiliza la función cv2.undistort.

# Imagen de entrada
Iorig = cv.imread('calib_camara/14.jpg')

# Corregir la distorsión
Iundist = cv.undistort(Iorig, M, coefs_dist, None)

# Visualizar
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20,10))
ax1.imshow(Iorig)
ax1.set_title('Imagen original', fontsize=20); ax1.axis('off')
ax2.imshow(Iundist)
ax2.set_title('Imagen sin distorsión', fontsize=20); ax2.axis('off');




