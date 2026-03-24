#include "temp_condition_filter/temp_condition_filter.hpp"

TempConditionFilter::TempConditionFilter()
{
    this->duration_ = std::chrono::milliseconds(1000);
}

TempConditionFilter::TempConditionFilter(uint32_t delay_to_trigger_ms)
{
    this->duration_ = std::chrono::milliseconds(delay_to_trigger_ms);
}

bool TempConditionFilter::trig(bool condition)
{
    auto now = Clock::now();

    if (condition)
    {
        if (!triggered_)
        {
            triggered_ = true;
            trig_time_ = now;
        }
        else
        {
            if (now - trig_time_ >= duration_)
            {
                return true;
            }
        }
    }
    else
    {
        triggered_ = false;
    }

    return false;
}

void TempConditionFilter::reset()
{
    triggered_ = false;
}

void TempConditionFilter::update_delay(uint32_t delay_to_trigger_ms)
{
    this->duration_ = std::chrono::milliseconds(delay_to_trigger_ms);
}