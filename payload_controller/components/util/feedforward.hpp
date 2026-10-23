#pragma once

class FeedForward {

    public:
        FeedForward(float out_for_fric, float out_per_in, float set_point) : out_for_fric(out_for_fric), out_per_in(out_per_in), set_point(set_point) {}

        float update(float deltaT);
        void new_set_point(float new_set_point);
    
    private:
        float out_for_fric;
        float out_per_in;

        float set_point;
};