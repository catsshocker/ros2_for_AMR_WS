#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_base/lib/updown_speed.hpp"

enum class move_type{
    MOVE,
    TURN,
    MOVE_AND_TURN,
};


class MovvveTask{
public:
    MovvveTask(
        double dx,
        double dy,
        double dtheta,
        double Vm,
        double Acc,
        double dt
    );
    bool finished();
    geometry_msgs::msg::Twist update();
private:
    double l,dtheta,dt,Vm,Acc;
    int i;
    double now_speed,move_angle,car_angle;
    bool is_finished;
    UpDownSpeed updown_speed;
    move_type move_type_;

};

