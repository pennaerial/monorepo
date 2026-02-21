#ifndef GPIO_HPP
#define GPIO_HPP

#include <rclcpp/rclcpp.hpp>


enum class PinType : uint8_t {
    Binary = 1 << 0, // 00000001
    PWM =    1 << 1, // 00000010
    Servo =  1 << 2  // 00000100
};

constexpr PinType operator|(PinType a, PinType b) {
    return static_cast<PinType>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

class GPIO {
    public:
        GPIO(int handle, int pin, uint8_t permissions);
        void write_high();
        void write_low();
        void write_pwm(float frequency, float duty_cycle, int offset, int cycles);
        void write_servo(int pulse_width, int frequency, int offset, int cycles);

    private:
        int handle_{}; //handle for gpiochip
        int pin_{};
        uint8_t permissions_{};
        
        static rclcpp::Logger logger();
        bool allowed(PinType pin_type);

};

#endif