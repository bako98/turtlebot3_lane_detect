import cv2
import numpy as np
import matplotlib.pyplot as plt

# 이미지 불러오기
img_path = "/home/seokhwan/Pictures/Screenshots/Screenshot from 2025-07-03 01-09-38.png"
gray = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

# 이진화
_, mask = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)

# 컨투어 검출 및 필터링
contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
filtered = [cnt for cnt in contours if 500 < cv2.contourArea(cnt) < 20000]

# 면적 기준 내림차순 정렬
filtered = sorted(filtered, key=cv2.contourArea, reverse=True)

# 마스크 복사본 만들기
mask_split = mask.copy()

if filtered:
    largest = filtered[0]
    area = cv2.contourArea(largest)
    if area >= 17000:
        # 중심 x 계산
        temp_mask = np.zeros_like(mask)
        cv2.drawContours(temp_mask, [largest], -1, 255, -1)
        ys, xs = np.nonzero(temp_mask)
        mean_x = int(np.mean(xs))

        # 굵은 선으로 나누기 (20픽셀)
        mask_split[:, mean_x - 10:mean_x + 10] = 0

        print(f"[Split] area: {area:.1f}, mean_x: {mean_x}")

# 결과 디버그 이미지 생성
debug_img = cv2.cvtColor(mask_split, cv2.COLOR_GRAY2BGR)
contours, _ = cv2.findContours(mask_split, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

for cnt in contours:
    area = cv2.contourArea(cnt)
    if 500 < area < 20000:
        cv2.drawContours(debug_img, [cnt], -1, (0, 255, 255), 2)
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.putText(debug_img, f"Area: {int(area)}", (cx - 40, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

# 17000up 표시
if filtered and cv2.contourArea(filtered[0]) >= 17000:
    M = cv2.moments(filtered[0])
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        cv2.putText(debug_img, "17000up", (cx - 50, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

# 시각화
plt.figure(figsize=(10, 6))
plt.imshow(cv2.cvtColor(debug_img, cv2.COLOR_BGR2RGB))
plt.title("Filtered and Split Contours with '17000up'")
plt.axis("off")
plt.show()
