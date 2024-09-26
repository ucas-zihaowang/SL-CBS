
#ifndef TIMER_H_
#define TIMER_H_

#include <chrono>
typedef std::chrono::high_resolution_clock high_resolution_clock;
typedef std::chrono::milliseconds milliseconds;
typedef std::chrono::microseconds microseconds;


class Timer
{
public:


    explicit Timer(bool run = false)
    {
        if (run) Reset();
    }


    void Reset()
    {
        start = high_resolution_clock::now();
    }


    double Elapsed() const
    {
        return static_cast<double>( std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - start).count() );
    }

    double Elapsed_us() const
    {
        return static_cast<double>( std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start).count() );
    }

private:

    high_resolution_clock::time_point start;
};

#endif
