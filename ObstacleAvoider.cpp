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
       this->_robot.bump_update();
       if (this->_robot.is_bump()) {
           this->_state = BUMP;
           if (this->_robot.is_left_bump()) {
               this->_context.obstacle_direction = LEFT;
           } else if (this->_robot.is_front_bump()) {
               this->_context.obstacle_direction = FORWARD;
           } else if (this->_robot.is_right_bump()) {
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
        this->_state = GO;
    }

    void ObstacleAvoider::forward()
    {
        this->_robot.forward(40);
    }

    void ObstacleAvoider::bump()
    {
        switch(this->_context.obstacle_direction) {
            case LEFT:
                this->_robot.backward_meters_now(40, .15);
                this->_robot.spin_degrees_now(LEFT, 40, 60);
                this->_state = GO;
				break;
            case RIGHT:
                this->_robot.backward_meters_now(40, .15);
                this->_robot.spin_degrees_now(RIGHT, 40, 60);
                this->_state = GO;
				break;
            case FORWARD:
                this->_robot.backward_meters_now(40, .15);
                this->_robot.spin_degrees_now(LEFT, 40, 60);
                this->_state = GO;
				break;
            default:
                this->_state = GO;
				break;
        }
    }

    void ObstacleAvoider::run() 
    {
        switch(this->_state) {
            case START:
                Serial.println("START");
                this->start();
                break;
            case GO:
                Serial.println("GO");
                this->forward();
                break;
            case BUMP:
                Serial.println("BUMP");
                this->bump();
                break;
            case STOP:
                Serial.println("STOP");
                this->_robot.stop();
                break;
            default:
                Serial.println("DEFAULT");
                this->_robot.stop();
                break;
        }
        this->detect();
    }

}
