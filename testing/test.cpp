// #include <cmath> // Include for trigonometric functions
#include <bits/stdc++.h>
struct Quaternion {
    double w, x, y, z;
};

Quaternion rpyToQuaternion(double roll, double pitch, double yaw) {
    Quaternion quat;

    // Calculate half angles
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    // Calculate quaternion elements
    quat.w = cr * cp * cy + sr * sp * sy;
    quat.x = sr * cp * cy - cr * sp * sy;
    quat.y = cr * sp * cy + sr * cp * sy;
    quat.z = cr * cp * sy - sr * sp * cy;

    return quat;
}

int main(){
    Quaternion q = rpyToQuaternion(0, 0, 0);
    std::cout << q.x << " " << q.y << " " << q.z << " " << q.w << std::endl;
}

  