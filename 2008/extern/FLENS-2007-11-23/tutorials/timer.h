#include <ctime>

struct timer
{
    void
    tic()
    {
        _time = std::clock();
    }

    double
    toc() const
    {
        return  double(std::clock() - _time) / CLOCKS_PER_SEC;
    }

    std::clock_t _time;
};
