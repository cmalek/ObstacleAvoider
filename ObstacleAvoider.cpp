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
           Serial.print("  detect:IS BUMP:");
           this->transition(BUMP);
           if (this->_robot.is_left_bump()) {
               Serial.println("LEFT");
               this->_context.obstacle_direction = LEFT;
           } else if (this->_robot.is_front_bump()) {
               Serial.println("FORWARD");
               this->_context.obstacle_direction = FORWARD;
           } else if (this->_robot.is_right_bump()) {
               Serial.println("RIGHT");
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
                Serial.println("state:START");
                break;
            case GO:
                Serial.println("state:GO");
                break;
            case BUMP:
                break;
            case BACKUP:
                Serial.println("state:BACKUP");
                break;
            case SPIN:
                Serial.println("state:SPIN");
                break;
            case STOP:
                Serial.println("state:STOP");
                break;
            default:
                Serial.println("state:UNKNOWN");
                break;
        }
    }

    void ObstacleAvoider::backup()
    {
        if (this->_robot.is_done_moving()) {
            Serial.print("  backup:DONE MOVING");
            Serial.println(this->_context.obstacle_direction);
            this->transition(SPIN);
            switch(this->_context.obstacle_direction) {
                case LEFT:
                    Serial.println("    backup:SPIN RIGHT 45");
                    this->_robot.spin_degrees(RIGHT, 40, 45);
                    break;
                case RIGHT:
                    Serial.println("    backup:SPIN LEFT 45");
                    this->_robot.spin_degrees(LEFT, 40, 45);
                    break;
                case FORWARD:
                    Serial.println("    backup:SPIN LEFT 180");
                    this->_robot.spin_degrees(LEFT, 40, 180);
                    break;
                default:
                    Serial.println("    backup:DEFAULT:GO");
                    this->transition(GO);
                    break;
             }
        }
    }

    void ObstacleAvoider::spin()
    {
        if (this->_robot.is_done_spinning()) {
            Serial.println("  spin:DONE SPINNING");
            this->_context.obstacle_direction = NONE;
            this->transition(GO);
        }
    }

    void ObstacleAvoider::bump()
    {
        if (this->_context.obstacle_direction != NONE) {
            Serial.println("  bump:BACKUP");
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
