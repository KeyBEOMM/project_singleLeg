#include <iostream>
#include <Eigen/Dense> // 이게 빨간줄이 안 뜨고 잘 빌드되어야 성공!

int main() {
    // 3x1 벡터 정의
    Eigen::Vector3d v(1, 2, 3);
    
    // 3x3 행렬 정의
    Eigen::Matrix3d m;
    m << 1, 0, 0,
         0, 1, 0,
         0, 0, 1; // 단위행렬

    // 행렬 * 벡터 연산
    Eigen::Vector3d result = m * v;

    std::cout << "Eigen Linking Test Success!" << std::endl;
    std::cout << "Result Vector: \n" << result << std::endl;

    return 0;
}