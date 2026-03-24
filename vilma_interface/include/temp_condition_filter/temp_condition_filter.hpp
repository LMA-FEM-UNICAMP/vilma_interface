#ifndef TEMP_CONDITION_FILTER__HPP_
#define TEMP_CONDITION_FILTER__HPP_

#include <chrono>

class TempConditionFilter
{
public:
    using Clock = std::chrono::steady_clock;

    TempConditionFilter();
    TempConditionFilter(uint32_t delay_to_trigger_ms);

    bool trig(bool condition);

    void reset();

    void update_delay(uint32_t delay_to_trigger_ms);

private:
    std::chrono::duration<double> duration_;

    Clock::time_point trig_time_;
    
    bool triggered_;
};

#endif // TEMP_CONDITION_FILTER__HPP_
