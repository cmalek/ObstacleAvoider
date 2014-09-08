#ifndef ObstacleAvoider_h
#define ObstacleAvoider_h

#ifdef DEBUG
#define debug(msg) Serial.println(msg)
#else
#define debug(msg)
#endif

#include <Servo.h>
#include <NewPing.h>
#include <Rolley.h>

namespace rolley 
{
    // States
    enum states_t {
        START,
        GO,
        BUMP,
        STOP,
        SPIN,
        BACKUP
    };

    struct context_t {
        directions_t obstacle_direction;
        directions_t desired_direction;
    };


    class ObstacleAvoider 
    {
        public:
            ObstacleAvoider();
            void setup(Servo *, NewPing *);
            void run();

        private:
            void transition(states_t);
            void detect();
            void detect_bump();
            void start();
            void forward();
            void bump();
            void backup();
            void spin();
            states_t _state;
            context_t _context;
            rolley::Rolley _robot;
    };
}
#endif
