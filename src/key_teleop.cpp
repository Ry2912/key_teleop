#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <termios.h>
#include <unistd.h>

class KeyboardPublisher : public rclcpp::Node
{
public:
    KeyboardPublisher()
        : Node("key_teleop"), linear_x_(0.0), angular_z_(0.0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        std::cout << "  w\n";
        std::cout << "a s d\n";
        std::cout << "  x\n";

        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(100),
        //     std::bind(&KeyboardPublisher::timer_callback, this));
    }

    void spin_some()
    {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(this->shared_from_this());
        while (rclcpp::ok())
        {
            switch (getch())
            {
            case 'w':
                linear_x_ = 0.3;
                angular_z_ = 0.0;
                publish_message();
                break;
            case 'x':
                linear_x_ = -0.3;
                angular_z_ = 0.0;
                publish_message();
                break;
            case 'a':
                linear_x_ = 0.0;
                angular_z_ = 0.3;
                publish_message();
                break;
            case 'd':
                linear_x_ = 0.0;
                angular_z_ = -0.3;
                publish_message();
                break;
            case 's':
                linear_x_ = 0.0;
                angular_z_ = 0.0;
                publish_message();
                break;
            default:
                break;
            }
            exec.spin_some(std::chrono::milliseconds(100));
        }
    }

private:
    // void timer_callback()
    // {
    //     auto message = geometry_msgs::msg::Twist();
    //     message.linear.x = linear_x_;
    //     message.angular.z = angular_z_;
    //     publisher_->publish(message);
    // }
       
    void publish_message()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = linear_x_;
        message.angular.z = angular_z_;
        publisher_->publish(message);
    }

    int getch()
    {
        struct termios oldt, newt;
        int ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }



    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    double linear_x_;
    double angular_z_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardPublisher>();
    node->spin_some();
    rclcpp::shutdown();
    return 0;
}