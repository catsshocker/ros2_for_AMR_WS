#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <algorithm>

using namespace std::chrono_literals;

#define PI 3.14159265358979323846

constexpr double Ms_TO_DEGs = 3600.0 / PI;  // Conversion factor from m/s to degrees per second (deg/s) for a wheel with a radius of 1 meters
constexpr double Rads_TO_DEGs = 3.0 * 180.0 / PI; // Conversion factor from radians per second to motor degrees per second



class DCspeed: public rclcpp::Node{
public:
    DCspeed() : Node("DCspeed_node"){
        
        this->declare_parameter<double>("linear_factor", 1.0);
        this->declare_parameter<double>("angular_factor", 1.0);
        this->declare_parameter<double>("max_motor_speed", 480.0);  // Declare a parameter for maximum motor speed deg/s

        updateParameters();

        parameter_event_sub_ = this->add_on_set_parameters_callback(
            std::bind(&DCspeed::parameterEventCallback, this, std::placeholders::_1)
        );

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&DCspeed::callback,this,std::placeholders::_1)
        );

        motor_speed_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/speed_cmd", 10);
        RCLCPP_INFO(this->get_logger(), "DCspeed is running");
        
    }

private :
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_speed_pub_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_event_sub_;

    double linear_factor_;
    double angular_factor_;
    double max_motor_speed_;


    void callback(const geometry_msgs::msg::Twist::SharedPtr msg){  // callback function for /cmd_vel to wheel speed
        double Vx = msg->linear.x;    // get x speed
        double Vy = msg->linear.y;    // get y speed
        double angular = msg->angular.z;    // get angle speed(anguler speed)

        double speed_m1 = -0.8660254038 * Vx * Ms_TO_DEGs*linear_factor_ + 0.5 * Vy * Ms_TO_DEGs*linear_factor_ + angular * Rads_TO_DEGs*angular_factor_;
        double speed_m2 =  -Vy * Ms_TO_DEGs*linear_factor_ + angular * Rads_TO_DEGs*angular_factor_;
        double speed_m3 =  0.8660254038 * Vx * Ms_TO_DEGs*linear_factor_ + 0.5 * Vy * Ms_TO_DEGs*linear_factor_ + angular * Rads_TO_DEGs*angular_factor_;
        RCLCPP_INFO(this->get_logger(), "%.2f, %.2f, %.2f | %.2f, %.2f, %.2f",Vx,Vy,angular,speed_m1,speed_m2,speed_m3);

        if(std::abs(speed_m1) > max_motor_speed_ || std::abs(speed_m2) > max_motor_speed_ || std::abs(speed_m3) > max_motor_speed_){
            RCLCPP_WARN(this->get_logger(), "Calculated motor speeds exceed maximum limits. Clamping to max speed.");
        }
        setMotorSpeed(speed_m1, speed_m2, speed_m3);
    }

    void setMotorSpeed(double speed0, double speed1, double speed2){
        speed0 = std::clamp(speed0,-max_motor_speed_,max_motor_speed_);
        speed1 = std::clamp(speed1,-max_motor_speed_,max_motor_speed_);
        speed2 = std::clamp(speed2,-max_motor_speed_,max_motor_speed_);
        RCLCPP_INFO(this->get_logger(), "motor speeds set : %.2f, %.2f, %.2f", speed0, speed1, speed2);
        std_msgs::msg::Float64MultiArray motor_speeds;
        motor_speeds.data = {speed0, speed1, speed2};
        motor_speed_pub_->publish(motor_speeds);
    }

    void updateParameters() {
        linear_factor_ = this->get_parameter("linear_factor").as_double();
        angular_factor_ = this->get_parameter("angular_factor").as_double();
        max_motor_speed_ = this->get_parameter("max_motor_speed").as_double();
    }

    rcl_interfaces::msg::SetParametersResult parameterEventCallback(const std::vector<rclcpp::Parameter> &parameters) {
        for (const auto &param : parameters) {
            if (param.get_name() == "linear_factor") {
                linear_factor_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated linear_factor to: %.2f", linear_factor_);
            } else if (param.get_name() == "angular_factor") {
                angular_factor_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated angular_factor to: %.2f", angular_factor_);
            } else if (param.get_name() == "max_motor_speed") {
                max_motor_speed_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated max_motor_speed to: %.2f", max_motor_speed_);
            }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

};

int main(int argc,char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<DCspeed>());
    rclcpp::shutdown();
    return 0;
}