#ifndef t_on_filter__HPP_
#define t_on_filter__HPP_

#include <chrono>

class TOnFilter
{
public:
    using Clock = std::chrono::steady_clock;

    TOnFilter();
    TOnFilter(uint32_t delay_to_trigger_ms);

    bool trig(bool condition);

    void reset();

    void update_delay(uint32_t delay_to_trigger_ms);

private:
    std::chrono::duration<double> duration_;

    Clock::time_point trig_time_;
    
    bool triggered_;
};

#endif // t_on_filter__HPP_
