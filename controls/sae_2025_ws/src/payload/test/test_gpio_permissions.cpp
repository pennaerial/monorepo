#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "payload/gpio.hpp"
#include "mock_lgpio.hpp"

// Convenience: build a permissions byte from PinType flags
static uint8_t perm(std::initializer_list<PinType> types)
{
    uint8_t v = 0;
    for (auto t : types) v |= static_cast<uint8_t>(t);
    return v;
}

// Helpers to query mock call log
static bool called(const std::string& fn)
{
    for (const auto& c : mock_lgpio_calls())
        if (c.fn == fn) return true;
    return false;
}
static bool not_called(const std::string& fn) { return !called(fn); }

// Check that handle + pin are forwarded correctly by a call
static bool called_with(const std::string& fn, int handle, int pin)
{
    for (const auto& c : mock_lgpio_calls())
        if (c.fn == fn && c.handle == handle && c.pin == pin) return true;
    return false;
}

// ---- Fixture ----

class GpioPermTest : public ::testing::Test {
protected:
    void SetUp() override { mock_lgpio_reset(); }
};

// ======== Binary-only ========

TEST_F(GpioPermTest, Binary_AllowsHigh)
{
    GPIO pin(0, 5, perm({PinType::Binary}));
    pin.write_high();
    EXPECT_TRUE(called("lgGpioWrite"));
}

TEST_F(GpioPermTest, Binary_AllowsLow)
{
    GPIO pin(0, 5, perm({PinType::Binary}));
    pin.write_low();
    EXPECT_TRUE(called("lgGpioWrite"));
}

TEST_F(GpioPermTest, Binary_BlocksPwm)
{
    GPIO pin(0, 5, perm({PinType::Binary}));
    pin.write_pwm(200, 50.0f, 0, 0);
    EXPECT_TRUE(not_called("lgTxPwm"));
}

TEST_F(GpioPermTest, Binary_BlocksServo)
{
    GPIO pin(0, 5, perm({PinType::Binary}));
    pin.write_servo(1500, 50, 0, 0);
    EXPECT_TRUE(not_called("lgTxServo"));
}

// ======== PWM-only ========

TEST_F(GpioPermTest, Pwm_AllowsPwm)
{
    GPIO pin(0, 5, perm({PinType::PWM}));
    pin.write_pwm(200, 50.0f, 0, 0);
    EXPECT_TRUE(called("lgTxPwm"));
}

TEST_F(GpioPermTest, Pwm_BlocksHigh)
{
    GPIO pin(0, 5, perm({PinType::PWM}));
    pin.write_high();
    EXPECT_TRUE(not_called("lgGpioWrite"));
}

TEST_F(GpioPermTest, Pwm_BlocksLow)
{
    GPIO pin(0, 5, perm({PinType::PWM}));
    pin.write_low();
    EXPECT_TRUE(not_called("lgGpioWrite"));
}

TEST_F(GpioPermTest, Pwm_BlocksServo)
{
    GPIO pin(0, 5, perm({PinType::PWM}));
    pin.write_servo(1500, 50, 0, 0);
    EXPECT_TRUE(not_called("lgTxServo"));
}

// ======== Servo-only ========

TEST_F(GpioPermTest, Servo_AllowsServo)
{
    GPIO pin(0, 5, perm({PinType::Servo}));
    pin.write_servo(1500, 50, 0, 0);
    EXPECT_TRUE(called("lgTxServo"));
}

TEST_F(GpioPermTest, Servo_BlocksHigh)
{
    GPIO pin(0, 5, perm({PinType::Servo}));
    pin.write_high();
    EXPECT_TRUE(not_called("lgGpioWrite"));
}

TEST_F(GpioPermTest, Servo_BlocksLow)
{
    GPIO pin(0, 5, perm({PinType::Servo}));
    pin.write_low();
    EXPECT_TRUE(not_called("lgGpioWrite"));
}

TEST_F(GpioPermTest, Servo_BlocksPwm)
{
    GPIO pin(0, 5, perm({PinType::Servo}));
    pin.write_pwm(200, 50.0f, 0, 0);
    EXPECT_TRUE(not_called("lgTxPwm"));
}

// ======== Combination: Binary | PWM ========

TEST_F(GpioPermTest, BinaryPwm_AllowsHighAndPwm)
{
    GPIO pin(0, 5, perm({PinType::Binary, PinType::PWM}));
    pin.write_high();
    pin.write_pwm(200, 50.0f, 0, 0);
    EXPECT_TRUE(called("lgGpioWrite"));
    EXPECT_TRUE(called("lgTxPwm"));
}

TEST_F(GpioPermTest, BinaryPwm_BlocksServo)
{
    GPIO pin(0, 5, perm({PinType::Binary, PinType::PWM}));
    pin.write_servo(1500, 50, 0, 0);
    EXPECT_TRUE(not_called("lgTxServo"));
}

// ======== No permissions ========

TEST_F(GpioPermTest, NoPerms_BlocksAll)
{
    GPIO pin(0, 5, 0);
    pin.write_high();
    pin.write_low();
    pin.write_pwm(200, 50.0f, 0, 0);
    pin.write_servo(1500, 50, 0, 0);
    EXPECT_TRUE(mock_lgpio_calls().empty());
}

// ======== All permissions ========

TEST_F(GpioPermTest, AllPerms_AllowsAll)
{
    GPIO pin(0, 5, perm({PinType::Binary, PinType::PWM, PinType::Servo}));
    pin.write_high();
    pin.write_pwm(200, 50.0f, 0, 0);
    pin.write_servo(1500, 50, 0, 0);
    EXPECT_TRUE(called("lgGpioWrite"));
    EXPECT_TRUE(called("lgTxPwm"));
    EXPECT_TRUE(called("lgTxServo"));
}

// ======== Passthrough: handle and pin forwarded correctly ========

TEST_F(GpioPermTest, Passthrough_BinaryForwardsHandleAndPin)
{
    GPIO pin(7, 23, perm({PinType::Binary}));
    pin.write_high();
    EXPECT_TRUE(called_with("lgGpioWrite", 7, 23));
}

TEST_F(GpioPermTest, Passthrough_PwmForwardsHandleAndPin)
{
    GPIO pin(7, 13, perm({PinType::PWM}));
    pin.write_pwm(200, 75.0f, 0, 0);
    EXPECT_TRUE(called_with("lgTxPwm", 7, 13));
}

TEST_F(GpioPermTest, Passthrough_ServoForwardsHandleAndPin)
{
    GPIO pin(7, 14, perm({PinType::Servo}));
    pin.write_servo(1500, 50, 0, 0);
    EXPECT_TRUE(called_with("lgTxServo", 7, 14));
}

// ---- Custom main: rclcpp must be initialized for RCLCPP_WARN in GPIO::allowed() ----
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
