import cv2
import numpy as np
import time

def nothing(x):
    pass

cap = cv2.VideoCapture(0)

# 降低分辨率，减少计算量，提高流畅度
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

cv2.namedWindow('Control')

# 颜色范围故意放宽（尤其是苹果的红色/橙红/偏黄都包含）
cv2.createTrackbar('H Lower', 'Control', 0, 180, nothing)
cv2.createTrackbar('H Upper', 'Control', 40, 180, nothing)   # 放宽到40，包含橙红
cv2.createTrackbar('S Lower', 'Control', 60, 255, nothing)   # 降低饱和度下限
cv2.createTrackbar('S Upper', 'Control', 255, 255, nothing)
cv2.createTrackbar('V Lower', 'Control', 40, 255, nothing)   # 降低亮度下限
cv2.createTrackbar('V Upper', 'Control', 255, 255, nothing)

last_process_time = 0
process_interval = 1.0  # 每 1 秒处理一次

while True:
    ret, frame = cap.read()
    if not ret:
        break

    current_time = time.time()

    # 只每秒处理一次
    if current_time - last_process_time < process_interval:
        cv2.imshow('Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue

    last_process_time = current_time

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    h_l = cv2.getTrackbarPos('H Lower', 'Control')
    h_u = cv2.getTrackbarPos('H Upper', 'Control')
    s_l = cv2.getTrackbarPos('S Lower', 'Control')
    s_u = cv2.getTrackbarPos('S Upper', 'Control')
    v_l = cv2.getTrackbarPos('V Lower', 'Control')
    v_u = cv2.getTrackbarPos('V Upper', 'Control')

    lower = np.array([h_l, s_l, v_l])
    upper = np.array([h_u, s_u, v_u])

    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    command = ""  # 最终要执行的命令

    if contours:
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        # 过滤太小的物体
        if area < 600:
            cv2.imshow('Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        perimeter = cv2.arcLength(largest, True)
        circularity = 4 * np.pi * area / (perimeter ** 2) if perimeter > 0 else 0

        x, y, w, h = cv2.boundingRect(largest)
        aspect = float(w) / h if h > 0 else 1.0

        # ========================
        #   核心判断逻辑（形态为主）
        # ========================

        # 苹果：圆度高 + 宽高比接近1
        if (circularity > 0.62 and          # 圆度放宽到0.62
            0.7 < aspect < 1.45 and         # 宽高比更宽松
            area > 600):                    # 面积要求稍高，避免误判
            command = "苹果 → 右转"
            color = (0, 255, 0)  # 绿色框

        # 香蕉：圆度低 + 宽高比很大
        elif (circularity < 0.40 and
              1.9 < aspect < 6.0 and
              area > 600):
            command = "香蕉 → 左转"
            color = (0, 165, 255)  # 橙色框（区分）

        if command:
            cv2.drawContours(frame, [largest], -1, color, 2)
            cv2.putText(frame, command, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            print(command)

    cv2.imshow('Detection', frame)
    cv2.imshow('Mask', mask)  # 想调试掩码时打开

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()