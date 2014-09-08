#include <ObstacleAvoider.h>

namespace rolley
{
    ObstacleAvoider::ObstacleAvoider() : 
        _state(START)
    {}

    void ObstacleAvoider::setup(Servo *servo, NewPing *sonar)  
    {
        this->_robot = rolley::Rolley();
        this->_robot.setup(servo, sonar);
    }

    void ObstacleAvoider::detect_bump()
    {
       if (this->_robot.is_bump()) {
           this->transition(BUMP);
           if (this->_robot.is_left_bump()) {
               debug("detect:IS BUMP:LEFT");
               this->_context.obstacle_direction = LEFT;
           } else if (this->_robot.is_front_bump()) {
               debug("detect:IS BUMP:FORWARD");
               this->_context.obstacle_direction = FORWARD;
           } else if (this->_robot.is_right_bump()) {
               debug("detect:IS BUMP:RIGHT");
               this->_context.obstacle_direction = RIGHT;
           }
       }
    }


    void ObstacleAvoider::detect()
    {
        this->detect_bump();
    }
    
    void ObstacleAvoider::start()
    {
        this->_robot.stop();
        this->_robot.servo_set_position(90);
        this->transition(GO);
    }

    void ObstacleAvoider::forward()
    {
        this->_robot.forward(100);
    }

    void ObstacleAvoider::transition(states_t state)
    {
        this->_state = state;
        switch (this->_state) {
            case START:
                debug("state:START");
                break;
            case GO:
                debug("state:GO");
                break;
            case BUMP:
                debug("state:BUMP");
                break;
            case BACKUP:
                debug("state:BACKUP");
                break;
            case SPIN:
                debug("state:SPIN");
                break;
            case STOP:
                debug("state:STOP");
                break;
            default:
                debug("state:UNKNOWN");
                break;
        }
    }

    void ObstacleAvoider::backup()
    {
        if (this->_robot.is_done_moving()) {
            debug("  backup:DONE MOVING");
            this->transition(SPIN);
            switch(this->_context.obstacle_direction) {
                case LEFT:
                    debug("    backup:SPIN RIGHT 45");
                    this->_robot.spin_degrees(RIGHT, 40, 45);
                    break;
                case RIGHT:
                    debug("    backup:SPIN LEFT 45");
                    this->_robot.spin_degrees(LEFT, 40, 45);
                    break;
                case FORWARD:
                    debug("    backup:SPIN LEFT 180");
                    this->_robot.spin_degrees(LEFT, 40, 180);
                    break;
                default:
                    debug("    backup:DEFAULT:GO");
                    this->transition(GO);
                    break;
             }
        }
    }

    void ObstacleAvoider::spin()
    {
        if (this->_robot.is_done_spinning()) {
            debug("  spin:DONE SPINNING");
            this->_context.obstacle_direction = NONE;
            this->transition(GO);
        }
    }

    void ObstacleAvoider::bump()
    {
        if (this->_context.obstacle_direction != NONE) {
            debug("  bump:BACKUP");
            this->_robot.backward_meters(100, .30);
            this->transition(BACKUP);
        } else {
            this->transition(GO);
        }
    }

    void ObstacleAvoider::run() 
    {
        this->_robot.compass_update();
        this->_robot.bump_update();
        switch(this->_state) {
            case START:
                this->start();
                break;
            case GO:
                this->forward();
                this->detect();
                break;
            case BUMP:
                this->bump();
                break;
            case BACKUP:
                this->backup();
                break;
            case SPIN:
                this->spin();
                break;
            case STOP:
                this->_robot.stop();
                break;
            default:
                this->_robot.stop();
                break;
        }
    }
}
