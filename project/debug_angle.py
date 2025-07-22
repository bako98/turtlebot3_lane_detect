import cv2
import numpy as np
import math

def process_image(image):
    # 1. 리사이즈
    resized = cv2.resize(image, (320, 240))

    # 2. 흑백 마스크 생성
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)

    # 3. 컨투어 검출
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f"contours 개수: {len(contours)}")

    if not contours:
        print("컨투어 없음")
        return resized

    # 4. 가장 큰 컨투어 선택
    largest_contour = max(contours, key=cv2.contourArea)

    # 5. 해당 컨투어로 마스크 생성
    mask_single = np.zeros_like(mask)
    cv2.drawContours(mask_single, [largest_contour], -1, 255, -1)

    ys, xs = np.nonzero(mask_single)

    if len(xs) == 0:
        print("픽셀 검출 실패")
        return resized

    # 6. 기울기 추정
    poly = np.polyfit(xs, ys, 2)
    x_center = np.mean(xs)
    slope = 2 * poly[0] * x_center + poly[1]

    # 7. 기울기 → 각도
    angle_rad = np.arctan(slope)
    angle_deg = np.degrees(angle_rad)
    print(f"가장 큰 contour 기울기(deg): {angle_deg:.2f}")

    # 8. 직선 시각화
    center = (int(x_center), int(np.polyval(poly, x_center)))
    length = 100
    dx = int(length * math.cos(angle_rad))
    dy = int(length * math.sin(angle_rad))

    pt1 = (center[0] - dx, center[1] - dy)
    pt2 = (center[0] + dx, center[1] + dy)

    debug_img = resized.copy()
    cv2.line(debug_img, pt1, pt2, (0, 0, 255), 2)
    cv2.circle(debug_img, center, 4, (255, 0, 0), -1)

    return debug_img

# 실행 예제
if __name__ == "__main__":
    img = cv2.imread("/home/seokhwan/Pictures/Screenshots/Screenshot from 2025-07-02 21-16-10.png")
    output = process_image(img)
    cv2.imshow("Debug Line", output)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
