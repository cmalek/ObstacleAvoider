#include <ObstacleAvoider.h>

namespace rolley
{
    ObstacleAvoider::ObstacleAvoider()  
    {}

    void ObstacleAvoider::setup(Servo *servo, NewPing *sonar)  
    {
         this->_robot = rolley::Rolley();
         this->_robot.setup(servo, sonar);
         this->_robot.sonar_set_wall_distance(WALL_DISTANCE);
         this->_robot.servo_set_scan_range(45, 135, 5);
         this->transition(START);
    }

    void ObstacleAvoider::reset_context() {
        this->_context.obstacle_direction = NONE;
        this->_context.desired_direction = NONE;
    }

    void ObstacleAvoider::detect()
    {
        float distance = 0.0;
        int servo_position = 0;

        this->_robot.servo_scan();
        if (this->_robot.is_bump()) {
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
            this->transition(BACKUP);
        } else if (this->_robot.is_sonar_wall()) {
            servo_position = this->_robot.servo_get_position();
            if (servo_position > 105) {
                debug("detect:IS SONAR:RIGHT");
                this->_context.desired_direction = LEFT;
            } else if (servo_position < 75) {
                debug("detect:IS SONAR:LEFT");
                this->_context.desired_direction = RIGHT;
            } else {
                debug("detect:IS SONAR:FORWARD");
                this->_context.desired_direction = LEFT;
            }
            distance = this->_robot.sonar_get_distance();
            if (distance < 18.0) {
                this->transition(BACKUP);
            } else {
                this->transition(SPIN);
            }
        }
    }

    void ObstacleAvoider::transition(states_t state)
    {
        this->_state = state;

        // onEnter events
        switch (this->_state) {
            case START:
                debug("state:START");
                this->_robot.stop();
                this->_robot.servo_set_position(90);
                this->transition(GO);
                break;
            case GO:
                debug("state:GO");
                this->reset_context();
                this->_robot.forward(FORWARD_SPEED);
                break;
            case BACKUP:
                debug("state:BACKUP");
                this->_robot.backward_meters(FORWARD_SPEED, .30);
                break;
            case SPIN:
                debug("state:SPIN");
                if (this->_context.desired_direction != NONE) {
                    switch(this->_context.desired_direction) {
                        case LEFT:
                            debug("  transition:SPIN:LEFT 45");
                            this->_robot.spin_degrees(LEFT, SPIN_SPEED, 45);
                            break;
                        case RIGHT:
                            debug("  transition:SPIN:RIGHT 45");
                            this->_robot.spin_degrees(RIGHT, SPIN_SPEED, 45);
                            break;
                        default:
                            debug("  transition:SPIN:DEFAULT:GO");
                            this->transition(GO);
                            break;
                    }
                } else {
                    switch(this->_context.obstacle_direction) {
                        case LEFT:
                            debug("  transition:SPIN:RIGHT 45");
                            this->_robot.spin_degrees(RIGHT, SPIN_SPEED, 45);
                            break;
                        case RIGHT:
                            debug("  transition:SPIN:LEFT 45");
                            this->_robot.spin_degrees(LEFT, SPIN_SPEED, 45);
                            break;
                        case FORWARD:
                            debug("  transition:SPIN:LEFT 90");
                            this->_robot.spin_degrees(LEFT, SPIN_SPEED, 90);
                            break;
                        default:
                            debug("  transition:SPIN:DEFAULT:GO");
                            this->transition(GO);
                            break;
                    }
                }
                break;
            case STOP:
                debug("state:STOP");
                this->_robot.stop();
                break;
            default:
                debug("state:UNKNOWN");
                this->_robot.stop();
                break;
        }
    }

    void ObstacleAvoider::backup()
    {
        if (this->_robot.is_done_moving()) {
            debug("  backup:DONE MOVING");
            this->transition(SPIN);
        }
    }

    void ObstacleAvoider::spin()
    {
        if (this->_robot.is_done_spinning()) {
            debug("  spin:DONE SPINNING");
            this->transition(GO);
        }
    }

    void ObstacleAvoider::run() 
    {
        this->_robot.compass_update();
        this->_robot.bump_update();
        switch(this->_state) {
            case START:
                break;
            case GO:
                this->detect();
                break;
            case BACKUP:
                this->backup();
                break;
            case SPIN:
                this->spin();
                break;
            case STOP:
                break;
            default:
                this->_robot.stop();
                break;
        }
    }
}
