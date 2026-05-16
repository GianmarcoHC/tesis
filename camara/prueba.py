import cv2

for i in range(5):
    cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
    if cap.isOpened():
        print(f"✅ Cámara {i} disponible")
        cap.release()
    else:
        print(f"❌ Cámara {i} no disponible")
