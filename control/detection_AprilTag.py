# apriltag_pose_logger.py
# Requisitos: pip install opencv-python numpy pupil-apriltags

import cv2 as cv, numpy as np, os, csv, time, socket, struct
from pupil_apriltags import Detector # es el detector robusto de los AprilTags.
from math import atan2, asin, degrees

# ===== UDP CONFIG =====
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ====== CONFIGURACIÓN INCIAL ======
CAM_INDEX   = 0          # 0 o 1 según el sistema
RESOLUTION  = (1920,1080) # (ancho, alto)(1920,1080)Full HD, (1280,720)HD
FPS         = 60
TAG_SIZE_M  = 0.067  # 6.7cm lado físico del tag (m). Medir cunado se ponga sobre la plataforma.
CSV_PATH    = "pose_log.csv" # aquí registramos los datos
TAG_FAMILY  = "tag36h11" # diccionario del tag que usaremos

# ===========================
# --- Offset opcional entre el centro del Tag y el centro del Efector (marco del Tag) ---
# Si el tag está perfectamente centrado/pegado alineado al efector, deja la identidad.
USE_OFFSET  = True # Se cambia  TRUE si no està centrado
t_te = np.array([0.00, 0.00, 0.00])   # traslación Tag->Efector (m)
# Rotación Tag->Efector (en grados, orden ZYX). Ej.: girar 90° sobre Z => [90,0,0]
eul_te_deg = [0.0, 0.0, 180.0]
# ===========================

def eul_zyx_to_rotm(eul_deg): # pasa de Euler ZYX (yaw–pitch–roll) a matriz de rotación. La usamos para construir R_te (rotación fija Tag-Efector)
    z, y, x = [np.deg2rad(a) for a in eul_deg]  # yaw(Z), pitch(Y), roll(X)
    cz, sz = np.cos(z), np.sin(z)
    cy, sy = np.cos(y), np.sin(y)
    cx, sx = np.cos(x), np.sin(x)
    Rz = np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])
    Ry = np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])
    Rx = np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])
    return Rz @ Ry @ Rx

def rotm_to_eul_zyx(R): # para converitr la matriz de rotación a ángulos roll, pitch, yaw (orden ZYX, en grados)
    sy = -R[2,0]
    sy = max(-1.0, min(1.0, sy))  # clamp por seguridad numérica
    yaw = atan2(R[1,0], R[0,0])
    pitch = asin(sy)
    roll = atan2(R[2,1], R[2,2])
    return degrees(roll), degrees(pitch), degrees(yaw)  # (roll, pitch, yaw)

R_te = eul_zyx_to_rotm(eul_te_deg)

# Carga calibración
K = None; dist = None # M y dist se cmabia por K si se usa los otros parametros_1 (K y dist ---- M y coefs_dist)
if os.path.exists("camera_params_2.npz"): # ver cuál se usa
    params = np.load("camera_params_2.npz", allow_pickle=True)
    K = params["M"]; dist = params["coefs_dist"]
    print("[OK] Calibración cargada de camera_params_2.npz")
else:
    print("[!] Sin calibración: se estimará intrínseca aproximada (z tendrá más error).")

# Detector AprilTag
detector = Detector(families=TAG_FAMILY,
                    nthreads=2, quad_decimate=1.0, quad_sigma=0.0,
                    refine_edges=True, decode_sharpening=0.25)

# Cámara
cap = cv.VideoCapture(CAM_INDEX, cv.CAP_DSHOW)
cap.set(cv.CAP_PROP_FRAME_WIDTH,  RESOLUTION[0])
cap.set(cv.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
cap.set(cv.CAP_PROP_FPS, FPS)



# Guardar parámetros en un .CSV para posterior análisis
new_file = not os.path.exists(CSV_PATH)
csv_file = open(CSV_PATH, "a", newline="")
writer = csv.writer(csv_file)
if new_file:
    writer.writerow(["timestamp_s","id",
                     "x_m","y_m","z_m",
                     "roll_deg","pitch_deg","yaw_deg",
                     "xE_m","yE_m","zE_m",
                     "rollE_deg","pitchE_deg","yawE_deg"])

print("Ventana activa. q = salir | g = guardar frame.")
while True:
    ok, frame = cap.read()
    if not ok: break
    ts = time.time()

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    h, w = gray.shape[:2]

    if K is None: # si K no existe, se usa la heurística
        fx = fy = 1.2 * max(w, h)
        cx, cy = w/2.0, h/2.0
        dist0 = np.zeros(5)
    else: # si K existe, se usa la intrínseca. El detector en si necesita intrínseca para reoslver PnP y dar la pose(pos,rot)
        fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
        dist0 = dist
    # detección:
    dets = detector.detect(gray, estimate_tag_pose=True, #instruye al detector a ejecutar PnP internamente con las esquinas 2D del tag, sus puntos 3D (derivados del tamaño) y la intrínseca 
                           #                             → te devuelve: det.pose_t y _det.pose_R
                           camera_params=(fx, fy, cx, cy),
                           tag_size=TAG_SIZE_M)

    out = frame.copy()
    for d in dets:
        # Dibujo básico
        crn = d.corners.astype(int)
        for i in range(4): cv.line(out, tuple(crn[i]), tuple(crn[(i+1)%4]), (0,255,0), 2)
        cX, cY = map(int, d.center)
        cv.circle(out, (cX,cY), 4, (0,0,255), -1)
        cv.putText(out, f"id:{d.tag_id}", (cX+8, cY-8), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        # Pose Tag respecto a Cámara
        t_tc = d.pose_t.reshape(3)  # (tx,ty,tz) en metros
        R_tc = d.pose_R

        roll, pitch, yaw = rotm_to_eul_zyx(R_tc)

        # --- Pose del EFECTOR (si hay offset Tag->Efector)
        if USE_OFFSET:
            R_ec = R_tc @ R_te          # R_ec: Efector respecto Cámara
            t_ec = R_tc @ t_te + t_tc   # t_ec: idem traslación
            rollE, pitchE, yawE = rotm_to_eul_zyx(R_ec)
            xe, ye, ze = t_ec

            # Para visualización
            R_vis = R_ec
            t_vis = t_ec

        else:
            R_ec = R_tc
            t_ec = t_tc
            rollE, pitchE, yawE = roll, pitch, yaw
            xe, ye, ze = t_tc

            # Para visualización
            R_vis = R_tc
            t_vis = t_tc

        # =============================
        # ====== ENVÍO UDP ============
        # =============================
        # Convertimos:
        #   metros → mm
        #   grados → miligrados
        xe_i = int(xe * 1000)
        ye_i = int((-ye) * 1000)
        ze_i = int((0.495 - ze + 0.033) * 1000)
        rollE_i  = int((rollE + 180 if rollE < 0 else rollE - 180)  * 10)
        pitchE_i = int((-pitchE) * 10)
        yawE_i   = int((-yawE)   * 10)

        # Empaquetado en 6 enteros int16
        msg = struct.pack("<6h",
                          xe_i, ye_i, ze_i,
                          rollE_i, pitchE_i, yawE_i)

        sock.sendto(msg, (UDP_IP, UDP_PORT))
        # =============================
        # Valores para visualizacion:
        xe_viz = float(xe_i / 1000)
        ye_viz = float(ye_i / 1000)
        ze_viz = float(ze_i / 1000)
        rollE_viz  = float(rollE_i / 10)
        pitchE_viz = float(pitchE_i / 10)
        yawE_viz   = float(yawE_i  / 10)

        # Proyección de ejes (visual)
        axis = np.float32([[0,0,0],
                           [TAG_SIZE_M*0.4,0,0],
                           [0,TAG_SIZE_M*0.4,0],
                           [0,0,TAG_SIZE_M*0.4]])

        rvec, _ = cv.Rodrigues(R_vis)
        imgpts, _ = cv.projectPoints(axis, rvec, t_vis,
                                    np.array([[fx,0,cx],[0,fy,cy],[0,0,1]], dtype=float),
                                    dist0)

        p0, px, py, pz = [tuple(np.int32(p.ravel())) for p in imgpts]
        cv.line(out, p0, px, (0,0,255), 3)
        cv.line(out, p0, py, (0,255,0), 3)
        cv.line(out, p0, pz, (255,0,0), 3)

        # Superposición de texto
        cv.putText(out, f"x={xe_viz:+.3f} y={ye_viz:+.3f} z={ze_viz:+.3f} m",
                   (20,30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        cv.putText(out, f"roll={rollE_viz:+.1f} pitch={pitchE_viz:+.1f} yaw={yawE_viz:+.1f} deg",
                   (20,55), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        # Log CSV (ambas poses: Tag y Efector)
        writer.writerow([f"{ts:.6f}", d.tag_id,
                         f"{t_tc[0]:.6f}", f"{t_tc[1]:.6f}", f"{t_tc[2]:.6f}",
                         f"{roll:.3f}", f"{pitch:.3f}", f"{yaw:.3f}",
                         f"{xe:.6f}", f"{ye:.6f}", f"{ze:.6f}",
                         f"{rollE:.3f}", f"{pitchE:.3f}", f"{yawE:.3f}"])

    cv.namedWindow("AprilTag Pose Logger", cv.WINDOW_NORMAL)
    cv.resizeWindow("AprilTag Pose Logger", 960, 540)


    cv.imshow("AprilTag Pose Logger", out)
    k = cv.waitKey(1) & 0xFF
    if k == ord('q'):
        break
    if k == ord('g'):
        fn = f"frame_{int(time.time())}.png"
        cv.imwrite(fn, out); print("Guardado:", fn)

csv_file.close()
cap.release()
cv.destroyAllWindows()
print("Listo. CSV ->", CSV_PATH)
