import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D

# 초기값 설정
init_x = 5
init_y = 4
init_z = 3

# 피타고라스 거리 계산 함수
def calc_yz_dist(y, z):
    return np.sqrt(y**2 + z**2)

# 플롯 설정
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(left=0.1, bottom=0.25)

# 시각화 업데이트 함수
def update(val):
    x = s_x.val
    y = s_y.val
    z = s_z.val
    
    ax.clear()
    
    # 1. 엔드이펙터 점 (파란색)
    ax.scatter(x, y, z, c='blue', s=100, label='End Effector (x,y,z)')
    
    # 2. YZ 평면에 투영된 점 (빨간색) - 그림자
    ax.scatter(0, y, z, c='red', s=100, label='Projected on YZ (0,y,z)')
    
    # 3. 선 그리기
    # 원점 -> 투영점 (우리가 구하려는 거리, 굵은 빨간 선)
    ax.plot([0, 0], [0, y], [0, z], 'r-', linewidth=3, label=f'Shadow Length: {calc_yz_dist(y,z):.2f}')
    
    # 투영점 -> 엔드이펙터 (점선, x축 거리)
    ax.plot([0, x], [y, y], [z, z], 'k--', alpha=0.5, label='Distance from Plane (x)')
    
    # Y성분, Z성분 보조선 (그림자 구성을 보여줌)
    ax.plot([0, 0], [0, y], [z, z], 'g:', alpha=0.5) # 가로
    ax.plot([0, 0], [y, y], [0, z], 'g:', alpha=0.5) # 세로
    
    # 축 설정 (고정해야 움직임이 잘 보임)
    ax.set_xlim([0, 10])
    ax.set_ylim([0, 10])
    ax.set_zlim([0, 10])
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_title(f'YZ Projection Distance = sqrt(y^2 + z^2)')
    ax.legend()

# 슬라이더 만들기
ax_x = plt.axes([0.2, 0.1, 0.65, 0.03])
ax_y = plt.axes([0.2, 0.06, 0.65, 0.03])
ax_z = plt.axes([0.2, 0.02, 0.65, 0.03])

s_x = Slider(ax_x, 'X Value', 0, 10, valinit=init_x)
s_y = Slider(ax_y, 'Y Value', 0, 10, valinit=init_y)
s_z = Slider(ax_z, 'Z Value', 0, 10, valinit=init_z)

# 슬라이더 변경 시 업데이트 함수 호출
s_x.on_changed(update)
s_y.on_changed(update)
s_z.on_changed(update)

# 초기 실행
update(None)

plt.show()