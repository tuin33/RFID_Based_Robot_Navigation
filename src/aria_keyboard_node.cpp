#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <cctype>
#include <iostream>
#include <thread>

class Operation{
private:
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    ros::Rate loop_rate = ros::Rate(5);  // 发布频率为5Hz
public:
    std::thread th1;
    geometry_msgs::Twist cmd_vel_msg;
    Operation(){
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 10);
        th1 = std::thread(&Operation::pubLoop, this);
    }
    void pubLoop(){
        
        while(ros::ok()){
            cmd_vel_pub.publish(cmd_vel_msg);
            loop_rate.sleep();
        }
    }
};

int kfd = 0;
struct termios cooked, raw;

void init_keyboard()
{
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
}

void restore_keyboard()
{
    tcsetattr(kfd, TCSANOW, &cooked);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_control");
    ros::NodeHandle nh;
    // ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 10);
    ros::Rate loop_rate(5);  // 发布频率为5Hz

    Operation operation;

    std::cout << "Keyboard node started." << std::endl;
    std::cout << "----------------------" << std::endl;
    std::cout << "i: move forward" << std::endl;
    std::cout << "m: move backward" << std::endl;
    std::cout << "j: turn left" << std::endl;
    std::cout << "k: turn right" << std::endl;
    std::cout << "z: add velocity" << std::endl;
    std::cout << "x: reduce velocity" << std::endl;
    std::cout << "s: stop" << std::endl;
    std::cout << "q: quit" << std::endl;
    std::cout << "----------------------" << std::endl;

    init_keyboard();

    double add_vel = 0.1;

    // geometry_msgs::Twist cmd_vel_msg;

    // cmd_vel_msg.linear.x = 0.05;

    // while(ros::ok()){
    //     cmd_vel_pub.publish(cmd_vel_msg);
    //     loop_rate.sleep();
    // }

    while (ros::ok())
    {
        // 发布速度指令
        // cmd_vel_pub.publish(cmd_vel_msg);

        // 休眠，控制发布频率
        loop_rate.sleep();
        
        // 读取键盘输入
        char c;
        if (read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        // 将字符转换为小写形式
        c = std::tolower(c);

        // 根据键盘输入设置速度指令
        switch (c)
        {
            case 'i':
                operation.cmd_vel_msg.linear.x += add_vel;
                std::cout << "Current linear velocity: " << operation.cmd_vel_msg.linear.x << std::endl;
                break;
            case 'm':
                operation.cmd_vel_msg.linear.x -= add_vel;
                std::cout << "Current linear velocity: " << operation.cmd_vel_msg.linear.x << std::endl;
                break;
            case 'j':
                operation.cmd_vel_msg.angular.z += add_vel;
                std::cout << "Current angular velocity: " << operation.cmd_vel_msg.angular.z << std::endl;
                break;
            case 'k':
                operation.cmd_vel_msg.angular.z -= add_vel;
                std::cout << "Current angular velocity: " << operation.cmd_vel_msg.angular.z << std::endl;
                break;
            case 'z':
                add_vel += 0.05;
                std::cout << "add_vel: " << add_vel << std::endl;
                break;
            case 'x':
                if(add_vel > 0.05){
                    add_vel -= 0.05;
                    std::cout << "add_vel: " << add_vel << std::endl;
                }
                break;
            case 's':
                operation.cmd_vel_msg.linear.x = 0;
                operation.cmd_vel_msg.angular.z = 0;
                std::cout << "Current linear velocity: " << operation.cmd_vel_msg.linear.x << std::endl;
                std::cout << "Current angular velocity: " << operation.cmd_vel_msg.angular.z << std::endl;
                break;
            case 'q':
                restore_keyboard();
                exit(0);
                break;
        }

        
    }

    restore_keyboard();

    return 0;
}