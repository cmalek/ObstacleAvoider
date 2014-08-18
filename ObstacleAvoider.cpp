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
        this->_robot.forward(20);
        delay(2000);
        this->_state = STOP;
    }

    void ObstacleAvoider::bump()
    {
        switch(this->_context.obstacle_direction) {
            case LEFT:
                this->_robot.backward_meters(100, .15);
                this->_robot.spin_degrees(RIGHT, 100, 30);
                this->_state = GO;
            case RIGHT:
                this->_robot.backward_meters(100, .15);
                this->_robot.spin_degrees(LEFT, 100, 30);
                this->_state = GO;
            default:
                this->_robot.stop();
        }
    }

    void ObstacleAvoider::run() 
    {
        switch(this->_state) {
            case START:
                this->start();
            case GO:
                this->forward();
            case BUMP:
                this->bump();
            case STOP:
                this->_robot.stop();
            default:
                this->_robot.stop();
        }
        //this->detect();
    }

}
