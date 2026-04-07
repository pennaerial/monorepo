#include "payload/encoder.hpp"

#include <cmath>

// Quadrature decode lookup table (4x).
// Index = (prev_ab << 2) | curr_ab  where ab = (A<<1)|B
// +1 = forward, -1 = reverse, 0 = no change or error
static const int8_t QEM[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0,
};

// TODO: Use GPIO classes instead
QuadratureEncoder::QuadratureEncoder(int pin_a, int pin_b, int cpr, MotorType motor_type)
: pin_a_(pin_a), pin_b_(pin_b), cpr_(cpr), motor_type_(motor_type)
{
    gpioSetMode(pin_a_, PI_INPUT);
    gpioSetMode(pin_b_, PI_INPUT);

    // Read initial state so the first edge is decoded correctly
    int a = gpioRead(pin_a_);
    int b = gpioRead(pin_b_);
    prev_ab_ = (a << 1) | b;

    // Register callback on each channel pin.
    gpioSetAlertFuncEx(pin_a_, alert_cb, this);
    gpioSetAlertFuncEx(pin_b_, alert_cb, this);
}

QuadratureEncoder::~QuadratureEncoder()
{
    gpioSetAlertFuncEx(pin_a_, nullptr, nullptr);
    gpioSetAlertFuncEx(pin_b_, nullptr, nullptr);
}

void QuadratureEncoder::alert_cb(int gpio, int level, uint32_t /*tick*/, void* userdata)
{
    auto* self = static_cast<QuadratureEncoder*>(userdata);
    if (!self) {
        return;
    }
    self->on_edge(gpio, level);
}

void QuadratureEncoder::on_edge(int gpio, int level)
{
    int a = (gpio == pin_a_) ? level : gpioRead(pin_a_);
    int b = (gpio == pin_b_) ? level : gpioRead(pin_b_);
    int curr_ab = (a << 1) | b;

    int8_t step = QEM[(prev_ab_ << 2) | curr_ab];
    if (motor_type_ == MotorType::LEFT) step = -step;
    count_.fetch_add(step, std::memory_order_relaxed);
    prev_ab_ = curr_ab;
}

int64_t QuadratureEncoder::count() const
{
    return count_.load(std::memory_order_relaxed);
}

void QuadratureEncoder::reset()
{
    count_.store(0, std::memory_order_relaxed);
}

float QuadratureEncoder::angle_deg() const
{
    return static_cast<float>(count()) * 360.0f / static_cast<float>(cpr_);
}

float QuadratureEncoder::angle_rad() const
{
    return static_cast<float>(count()) * (2.0f * static_cast<float>(M_PI)) / static_cast<float>(cpr_);
}
