import matplotlib.pyplot as plt
import numpy as np

# Error 값 범위 설정
error = np.linspace(-160, 160, 500)

# PD 제어 상수
Kp = 0.003
Kd = 0.01
RESTRICT_VEL = 0.26
MAX_VEL = 0.3
MAX_ANGULAR_Z = 1.0

last_error = 0  # 이전 에러는 0이라고 가정

# 각속도 그래프 (k 값별 변화)
k_ang_values = [1.0, 1.2]
colors_ang = ['blue', 'red']

angular_z_dict = {}
for k, color in zip(k_ang_values, colors_ang):
    scaled_error = np.sign(error) * (np.abs(error) ** k) / (160 ** k) * 160
    angular_z = Kp * scaled_error + Kd * (scaled_error - last_error)
    angular_z_clipped = np.where(
        angular_z < 0,
        -np.clip(np.abs(angular_z), 0, MAX_ANGULAR_Z),
        -np.clip(angular_z, 0, MAX_ANGULAR_Z)
    )
    angular_z_dict[f"k={k}"] = angular_z_clipped

# 그래프 그리기
plt.figure(figsize=(12, 5))

# (1) 선속도 그래프 (k값 여러 개)
plt.subplot(1, 2, 1)
k_values = [0.7, 1.0, 1.6, 2.2]
colors = ['red', 'green', 'blue', 'orange']
for k, color in zip(k_values, colors):
    linear_x = np.minimum(
        MAX_VEL * (np.maximum(1 - np.abs(error) / 160, 0) ** k),
        RESTRICT_VEL
    )
    plt.plot(error, linear_x, label=f'linear.x (k={k})', color=color)

plt.title("Error vs Linear Velocity (k 변화)")
plt.xlabel("Error (pixels)")
plt.ylabel("Linear Velocity (m/s)")
plt.grid(True)
plt.legend()

# (2) 각속도 그래프 (k별 비교)
plt.subplot(1, 2, 2)
for k_label, angular in angular_z_dict.items():
    plt.plot(error, angular, label=f'angular.z ({k_label})')

plt.title("Error vs Angular Velocity (k 변화)")
plt.xlabel("Error (pixels)")
plt.ylabel("Angular Velocity (rad/s)")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
