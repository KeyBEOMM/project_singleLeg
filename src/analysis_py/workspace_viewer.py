import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull

joint_limits = [
    [-0.7854, 2.0944], # HAA
    [-1.5708, 2.7925], # HFE
    [-2.705, 2.705]    # KFE
]


def skew_symmetric(w):
    """3D 벡터 w를 3x3 skew-symmetric 행렬 [w]로 변환"""
    return np.array([[0,    -w[2],  w[1]],
                     [w[2],  0,    -w[0]],
                     [-w[1], w[0],  0]])

def matrix_exponential(B, theta):
    """
    Twist B와 관절각 theta를 받아 SE(3) 변환 행렬을 반환 (Rodrigues' formula)
    C++의 exp_coordinate_Operator와 동일한 기능
    """
    T = np.eye(4)
    w = B[0:3]
    v = B[3:6]

    # 아주 작은 각도에 대한 예외 처리 (Pure Translation)
    if np.abs(theta) < 1e-9:
        T[0:3, 3] = v * theta
        return T

    bracket_omega = skew_symmetric(w)
    bracket_omega_sq = bracket_omega @ bracket_omega
    
    sin_t = np.sin(theta)
    cos_t = np.cos(theta)

    # 1. 회전 행렬 계산 (R)
    R = np.eye(3) + sin_t * bracket_omega + (1 - cos_t) * bracket_omega_sq
    
    # 2. 이동 벡터 계산 (G*v)
    # 교재 공식: (I*theta + (1-cos_t)*[w] + (theta-sin_t)*[w]^2) * v
    G_theta_v = (np.eye(3) * theta + (1 - cos_t) * bracket_omega + 
                 (theta - sin_t) * bracket_omega_sq) @ v

    T[0:3, 0:3] = R
    T[0:3, 3] = G_theta_v
    
    return T

def forward_kinematics_body(M_home, B_axes, joint_angles):
    """
    Body Frame PoE 알고리즘
    T = M * exp(B1*q1) * exp(B2*q2) * exp(B3*q3)
    """
    T_total = np.copy(M_home)
    
    # B_axes는 6x3 행렬 (각 열이 B1, B2, B3)
    for i in range(len(joint_angles)):
        B_i = B_axes[:, i]
        theta_i = joint_angles[i]
        
        T_temp = matrix_exponential(B_i, theta_i)
        T_total = T_total @ T_temp  # @는 행렬 곱셈 (C++의 *)
        
    return T_total

# --- 사용 예시 (Workspace Analysis) ---
if __name__ == "__main__":
    # 1. Home Configuration (예시: 다리를 아래로 뻗었을 때 발끝 위치)
    M = np.array([[1, 0, 0, 0.0],   # x 위치 0.1
                  [0, 1, 0, 0.04],
                  [0, 0, 1, -0.4],  # z 위치 -0.5 (아래방향)
                  [0, 0, 0, 1]])

    # 2. Body Screw Axes (B1, B2, B3) 설정 (예시 데이터)
    # 실제 로봇 설계에 맞게 6x3으로 구성하십시오.
    B = np.array([
        [1, 0, 0], # omega_x
        [0, 1, 1], # omega_y (HFE, KFE는 y축 회전)
        [0, 0, 0], # omega_z (예시)
        [0, -0.4, -0.19], # v_x
        [0.4, 0, 0], # v_y
        [0.04, 0, 0]  # v_z
    ])

    samples = 50000
    points = []

    for _ in range(samples):
        q = [np.random.uniform(l, u) for l, u in joint_limits]
        pos = forward_kinematics_body(M, B, q)
        points.append(pos[0:3, 3])  # End-effector position (x, y, z)

    points = np.array(points)

    # 2. 3D Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:,0], points[:,1], points[:,2], s=0.1, alpha=0.5)
    ax.set_xlabel('X (Front)')
    ax.set_ylabel('Y (Side)')
    ax.set_zlabel('Z (Down)')
    plt.show()

    hull = ConvexHull(points)

    # 3. 경계면의 정점(Vertices)들만 추출
    boundary_points = points[hull.vertices]

    # 4. 시각화 (경계면만 표시)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 전체 점은 흐리게, 경계 점은 명확하게
    ax.scatter(points[:,0], points[:,1], points[:,2], s=0.1, alpha=0.1, c='gray')
    ax.scatter(boundary_points[:,0], boundary_points[:,1], boundary_points
               [:,2], s=1, c='red')

    print(f"Total points: {len(points)}")
    print(f"Boundary points: {len(boundary_points)}")
    plt.show()

