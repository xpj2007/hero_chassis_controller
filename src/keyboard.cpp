#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "keyboard");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 关掉行缓冲和回显，按键立刻读到
    termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    ROS_INFO("w前进 s后退 a左移 d右移 q左转 e右转 空格停止");

    geometry_msgs::Twist cmd;
    const double speed = 0.5;   // m/s
    const double turn  = 1.0;   // rad/s

    ros::Rate rate(50);
    while (ros::ok())
    {
        // 非阻塞等 20ms，有键就处理
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        timeval tv{0, 20000};

        if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0)
        {
            char key = std::getchar();

            switch (key)
            {
            case 'w': cmd.linear.x =  speed; break;
            case 's': cmd.linear.x = -speed; break;
            case 'a': cmd.linear.y =  speed; break;
            case 'd': cmd.linear.y = -speed; break;
            case 'q': cmd.angular.z =  turn; break;
            case 'e': cmd.angular.z = -turn; break;
            case ' ': cmd = geometry_msgs::Twist(); break;
            case '\x03': ros::shutdown(); break;   // Ctrl-C
            default: continue;
            }
            pub.publish(cmd);

            cmd = geometry_msgs::Twist();
        }
        rate.sleep();
    }

    pub.publish(geometry_msgs::Twist());

    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
    return 0;
}