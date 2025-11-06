import cv2
import numpy as np
import socket
import math

# --- RTSP stream ---
url = "rtsp://ircbowling:ircbowling@172.27.196.26:554/stream1"
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
if not cap.isOpened():
    raise RuntimeError("❌ Could not open local camera (index 0).")

# --- Wi-Fi (UDP) setup ---
UDP_IP = "10.79.206.152"   # ← replace with ESP32 IP address
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- Load target image (template) ---
template = cv2.imread("target.jpg", cv2.IMREAD_GRAYSCALE)
if template is None:
    raise FileNotFoundError("❌ Could not load target.jpg – check path!")

th, tw = template.shape[::-1]  # width, height

# --- Parameters ---
threshold = 0.4  # detection confidence
scale_factors = np.linspace(0.5, 1.5, 10)  # template scale range
fov_degrees = 60  # ← camera field of view (adjust manually)

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Failed to grab frame, reconnecting…")
        cap.release()
        cap = cv2.VideoCapture(url)
        continue

    h, w = frame.shape[:2]
    cx = w // 2  # frame center X
    cv2.line(frame, (cx, 0), (cx, h), (0, 255, 0), 2)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    boxes = []
    confidences = []

    # --- Multi-scale template matching ---
    for scale in scale_factors:
        new_w, new_h = int(tw * scale), int(th * scale)

        if new_w < 10 or new_h < 10:
            continue
        if new_w > gray.shape[1] or new_h > gray.shape[0]:
            continue

        resized_template = cv2.resize(template, (new_w, new_h))
        res = cv2.matchTemplate(gray, resized_template, cv2.TM_CCOEFF_NORMED)
        loc = np.where(res >= threshold)

        for pt in zip(*loc[::-1]):
            boxes.append([pt[0], pt[1], new_w, new_h])
            confidences.append(float(res[pt[1], pt[0]]))

    # --- Merge overlapping detections ---
    idxs = cv2.dnn.NMSBoxes(boxes, confidences, threshold, 0.3)

    centers_x = []
    if len(idxs) > 0:
        for i in idxs.flatten():
            x, y, rw, rh = boxes[i]
            conf = confidences[i]
            top_left = (x, y)
            bottom_right = (x + rw, y + rh)

            cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)
            tx = x + rw // 2
            centers_x.append(tx)

        # --- Average detected centers ---
        avg_tx = int(np.mean(centers_x))
        dx = avg_tx - cx

        # Convert dx → angle (calibration handled by you)
        angle = (dx / w) * fov_degrees

        # Send only the number
        sock.sendto(str(round(angle, 2)).encode(), (UDP_IP, UDP_PORT))

        # Display info
        cv2.circle(frame, (avg_tx, h // 2), 6, (0, 0, 255), -1)
        cv2.putText(frame, f"Target FOUND dx={dx}px ang={angle:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "Target NOT FOUND", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    cv2.imshow("Tapo Scan", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
sock.close()

