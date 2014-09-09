#ifndef ObstacleAvoider_h
#define ObstacleAvoider_h

#define DEBUG
#ifdef DEBUG
#define debug(msg) Serial.println(msg)
#else
#define debug(msg) (void)0
#endif

#include <Servo.h>
#include <NewPing.h>
#include <Rolley.h>

#define WALL_DISTANCE 20
#define SPIN_SPEED 60
#define FORWARD_SPEED 60

namespace rolley 
{
    // States
    enum states_t {
        START,
        GO,
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
            void backup();
            void spin();
            void reset_context();
            

            states_t _state;
            context_t _context;

            rolley::Rolley _robot;
    };
}
#endif
