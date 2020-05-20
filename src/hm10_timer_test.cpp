#include "ros/ros.h"
#include <iostream>
#include <std_msgs/String.h>
#include <string>





int main(int argc, char **argv)
{
    ros::init(argc, argv, "hm10_test_timer");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    
    ros::Publisher command_pub = nh.advertise<std_msgs::String>("HM10/command", 1);
    
    uint8_t flag;

    uint32_t count = 0;
    while (ros::ok())
    {
        count++;
        std::cout << "[step] : " << count <<"mode:"<< std::endl;

        std_msgs::String msg;
        std::string prefix_major = "AT+MARJ0x";
        std::string prefix_minor = "AT+MINO0x";
        flag++;
        if(count > 0x1000)
        {
            count = 0;
        }
        if(flag % 2 == 0)
        {  
            int subfix_count = 0;
            std::string subfix_major = "";
            if(count <= 0x0010)
             subfix_count = 3;
             else if(count <= 0x0100)
             subfix_count = 2;
             else if(count <= 0x1000)
             subfix_count = 1;
            for(int i = 0; i < subfix_count; i++)
                subfix_major += "0";
            std::stringstream hexadecimal;
            hexadecimal << std::hex << std::uppercase << count;
            std::string pub_msg = prefix_major + subfix_major + hexadecimal.str();
            msg.data = pub_msg;
            std::cout << "pub_msg" << pub_msg <<std::endl;
        }
        else
        {
            int subfix_count = 0;
            std::string subfix_minor = "";
            if(count <= 0x0010)
             subfix_count = 3;
             else if(count <= 0x0100)
             subfix_count = 2;
             else if(count <= 0x1000)
             subfix_count = 1;
            for(int i = 0; i < subfix_count; i++)
                subfix_minor += "0";
            std::stringstream hexadecimal;
            hexadecimal << std::hex << std::uppercase << count;
            std::string pub_msg = prefix_minor + subfix_minor + hexadecimal.str();
            msg.data = pub_msg;
            std::cout << "pub_msg" << pub_msg <<std::endl;
        }
        
        command_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
