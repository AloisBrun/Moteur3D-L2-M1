#include "Timer.h"

Timer::Timer()
{
    //ctor
    start();
}

Timer::~Timer()
{
    //dtor
}

void Timer::start(){
    start_t = std::chrono::system_clock::now();
}
void Timer::end(){
    end_t = std::chrono::system_clock::now();
}


int Timer::getTime(int div){
    //int count = std::chrono::duration_cast<std::chrono::milliseconds>(end_t-start_t).count();
    int count = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-start_t).count();
    return count/div;
}
