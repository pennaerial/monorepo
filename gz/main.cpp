#include <gz/transport/Node.hh>
#include <gz/msgs/actuators.pb.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <string>



int main(){
    gz::transport::Node node;


    std::string model;
    std::cout << "Enter model: ";
    std::cin >> model;

    std::cout << "Enter number of motors: ";
    int num_motors{};
    std::cin >> num_motors;

    auto pub = node.Advertise<gz::msgs::Actuators>("/" + model + "/command/motor_speed");
    gz::msgs::Actuators motors;
    for (int i = 0; i < num_motors; i++){
	std::cout << "Speed for rotor_" + std::to_string(i) + ": ";
    	double speed{};
	std::cin >> speed;

    	motors.add_velocity(speed);
    }
    //motors.add_velocity(speed);    // rotor_0
    //motors.add_velocity(speed);    // rotor_1
    // motors.add_velocity(0.0);    // rotor_2
    // motors.add_velocity(0.0);    // rotor_3
    // motors.add_velocity(2500.0); // rotor_puller (main forward thrust)

    while (true)
    {
        std::cout << "published!\n";
        pub.Publish(motors);
        // std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 50 Hz
    }

    return 0;
}
