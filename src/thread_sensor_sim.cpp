#include <iostream>
#include <thread>
#include <chrono>

void sensorLoop() {
    while(true){
        std::cout << "Sensor reading...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

int main(){
    std::thread t(sensorLoop);
    t.join();
}
