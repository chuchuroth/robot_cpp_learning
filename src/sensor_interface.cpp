#include <iostream>

class Sensor {
public:
    virtual void read() = 0;
};

class Lidar : public Sensor {
public:
    void read() override {
        std::cout << "Reading lidar scan...\n";
    }
};

int main() {
    Sensor* s = new Lidar();
    s->read();
    delete s;
}
