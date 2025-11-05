#include <iostream>
#include <memory>

class Sensor {
public:
    virtual void open() = 0;
    virtual void read() = 0;
    virtual void close() = 0;
};

class Camera : public Sensor {
public:
    void open() override {
        std::cout << "Camera opened\n";
    }
    void read() override {
        std::cout << "Reading camera frame\n";
    }
    void close() override {
        std::cout << "Camera closed\n";
    }
};

int main() {
    std::unique_ptr<Sensor> cam = std::make_unique<Camera>();
    cam->open();
    cam->read();
    cam->close();
}
