#ifndef TIMER_H
#define TIMER_H

#include <chrono>
#include <thread>

class Timer
{
    public:
        Timer();
        virtual ~Timer();

        void start();
        void end();
        int getTime(int div=1);

        static void sleep(int time){
            std::chrono::milliseconds dura(time);
            std::this_thread::sleep_for( dura );
        }
    protected:
        std::chrono::time_point<std::chrono::system_clock> start_t;
        std::chrono::time_point<std::chrono::system_clock> end_t;
    private:
};

#endif // TIMER_H
