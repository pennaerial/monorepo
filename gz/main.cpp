#include <gz/transport/Node.hh>
#include <gz/msgs/actuators.pb.h>
#include <iostream>
#include <chrono>
#include <thread>
int main(){
    std::cout << "Hello world\n";
    gz::transport::Node node;
    auto pub = node.Advertise<gz::msgs::Actuators>("/standard_vtol_0/command/motor_speed");
    std::cout << "Publisher created. Sending motor commands..." << std::endl;
    gz::msgs::Actuators motors;
    motors.add_velocity(0.0);    // rotor_0
    motors.add_velocity(0.0);    // rotor_1
    motors.add_velocity(0.0);    // rotor_2
    motors.add_velocity(0.0);    // rotor_3
    motors.add_velocity(2100.0); // rotor_puller (main forward thrust)
    while (true)
    {
        std::cout << "published!\n";
        pub.Publish(motors);
        // std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 50 Hz
    }
    return 0;
}