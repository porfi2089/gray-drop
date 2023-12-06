#ifndef FUNCTIONS_H
#define FUNCTIONS_H

class Vector3 {
public:
    int x, y, z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(int x, int y, int z) : x(x), y(y), z(z) {}

    Vector3& add(const Vector3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vector3& subtract(const Vector3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vector3& multiply(const Vector3& other) {
            x *= other.x;
            y *= other.y;
            z *= other.z;
            return *this;
    }
};

float Calculate_Transformation_Matrix(float roll, float yaw, float pitch, int g, int p);

class Rocket_Functions{
        public:
            Vector3 position;
            Vector3 velocity;
            Vector3 acceleration;
            Vector3 rotation;
            Vector3 angular_velocity;
            Vector3 angular_acceleration;
        private:


};
#endif
