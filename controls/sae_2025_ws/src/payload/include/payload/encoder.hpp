#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <pigpiod_if2.h>
#include <atomic>
#include <cstdint>
#include "payload/motor.hpp"

// 4x quadrature decoder using pigpiod_if2 per-pin callbacks.
class QuadratureEncoder {
public:
    QuadratureEncoder(int pi, int pin_a, int pin_b, int cpr, MotorType motor_type);
    ~QuadratureEncoder();

    int64_t count()     const;  // raw 4x-decoded tick count
    float   angle_deg() const;  // degrees (unbounded)
    float   angle_rad() const;  // radians (unbounded)
    void    reset();            // zero the count

private:
    // Signature required by CBFuncEx_t (pigpiod_if2).
    static void alert_cb(int pi, unsigned gpio, unsigned level, uint32_t tick, void* userdata);
    void on_edge(unsigned gpio, unsigned level);

    int pi_;
    int pin_a_;
    int pin_b_;
    int cpr_;
    MotorType motor_type_;

    std::atomic<int64_t> count_{0};
    int prev_ab_{0};  // previous (A<<1|B) — only touched by pigpio callback thread

    int cbid_a_{-1};
    int cbid_b_{-1};
};

#endif // ENCODER_HPP
