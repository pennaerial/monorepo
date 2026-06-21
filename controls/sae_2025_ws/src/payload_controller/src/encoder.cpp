#include "payload_controller/encoder.hpp"

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

QuadratureEncoder::QuadratureEncoder(int pi, int pin_a, int pin_b, int cpr, MotorType motor_type)
: pi_(pi), pin_a_(pin_a), pin_b_(pin_b), cpr_(cpr), motor_type_(motor_type)
{
    set_mode(pi_, pin_a_, PI_INPUT);
    set_mode(pi_, pin_b_, PI_INPUT);

    int a = gpio_read(pi_, pin_a_);
    int b = gpio_read(pi_, pin_b_);
    prev_ab_ = (a << 1) | b;

    cbid_a_ = callback_ex(pi_, pin_a_, EITHER_EDGE, alert_cb, this);
    cbid_b_ = callback_ex(pi_, pin_b_, EITHER_EDGE, alert_cb, this);
}

QuadratureEncoder::~QuadratureEncoder()
{
    if (cbid_a_ >= 0) callback_cancel(cbid_a_);
    if (cbid_b_ >= 0) callback_cancel(cbid_b_);
}

void QuadratureEncoder::alert_cb(int /*pi*/, unsigned gpio, unsigned level, uint32_t /*tick*/, void* userdata)
{
    auto* self = static_cast<QuadratureEncoder*>(userdata);
    if (!self) return;
    self->on_edge(gpio, level);
}

void QuadratureEncoder::on_edge(unsigned gpio, unsigned level)
{
    int a = (gpio == static_cast<unsigned>(pin_a_)) ? static_cast<int>(level) : gpio_read(pi_, pin_a_);
    int b = (gpio == static_cast<unsigned>(pin_b_)) ? static_cast<int>(level) : gpio_read(pi_, pin_b_);
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
