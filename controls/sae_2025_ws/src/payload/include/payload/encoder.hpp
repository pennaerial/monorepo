#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <pigpio.h>
#include <atomic>
#include <cstdint>
#include "payload/motor.hpp"

// 4x quadrature decoder using pigpio per-pin alert callbacks.
class QuadratureEncoder {
public:
    // pin_a/b — channel A and B GPIO numbers (BCM)
    // cpr     — counts per revolution AFTER 4x decode (= 4 * encoder_ppr)
    QuadratureEncoder(int pin_a, int pin_b, int cpr, MotorType motor_type);
    ~QuadratureEncoder();

    int64_t count()     const;  // raw 4x-decoded tick count
    float   angle_deg() const;  // degrees (unbounded)
    float   angle_rad() const;  // radians (unbounded)
    void    reset();            // zero the count

private:
    // Signature required by gpioAlertFuncEx_t.
    static void alert_cb(int gpio, int level, uint32_t tick, void* userdata);
    void on_edge(int gpio, int level);

    int pin_a_;
    int pin_b_;
    int cpr_;
    MotorType motor_type_;

    std::atomic<int64_t> count_{0};
    int prev_ab_{0};  // previous (A<<1|B) — only touched by pigpio callback thread
};

#endif // ENCODER_HPP
