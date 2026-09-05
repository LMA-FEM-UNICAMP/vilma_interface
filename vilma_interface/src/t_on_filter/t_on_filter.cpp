#include "t_on_filter/t_on_filter.hpp"

TOnFilter::TOnFilter()
{
    this->duration_ = std::chrono::milliseconds(1000);
}

TOnFilter::TOnFilter(uint32_t delay_to_trigger_ms)
{
    this->duration_ = std::chrono::milliseconds(delay_to_trigger_ms);
}

bool TOnFilter::trig(bool condition)
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

void TOnFilter::reset()
{
    triggered_ = false;
}

void TOnFilter::update_delay(uint32_t delay_to_trigger_ms)
{
    this->duration_ = std::chrono::milliseconds(delay_to_trigger_ms);
}