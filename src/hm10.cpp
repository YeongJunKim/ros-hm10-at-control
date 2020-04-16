#include "ros/ros.h"
#include "Serial.h"
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <../include/imu.h>
#include <../include/AT.h>
#include <algorithm>
#include <string>
#include <unistd.h>
#include <time.h>
typedef struct
{
    // Euler
    float roll;
    float pitch;
    float yaw;

} Euler;

class HM10
{

public:
    HM10()
    {
        for (int i = 0; i < 1000; i++)
            buffer[i] = 0;

        dev = open_serial((char *)"/dev/ttyUSB1", 9600, 0, 0);

        Tx[0] = A;  // a
        Tx[1] = N;  // n
        Tx[2] = G;  // g
        Tx[3] = CR; // CR
        Tx[4] = LF; // LF
    }
    ~HM10()
    {
        close_serial(dev);
    }

    int set_beacon_mode(void)
    {
        delaytick(2000);
        sendAT(AT);
        delaytick(1000);
        readbuf();
        sendAT(RESET);
        delaytick(2000);
        readbuf();
        sendAT(AT);
        delaytick(1000);
        readbuf();
        sendAT(AT);
        delaytick(1000);
        readbuf();
        sendAT(AT);
        delaytick(500);
        readbuf();
        sendAT(MAJOR, "0X0001");
        delaytick(500);
        readbuf();
        sendAT(MINOR, "0X0002");
        delaytick(500);
        readbuf();
        sendAT(ADVERTISING_INTERVAL, "0");
        delaytick(500);
        readbuf();
        sendAT(NAME, "ACSL");
        delaytick(500);
        readbuf();
        sendAT(ADVERTISING_TYPE, "3");
        delaytick(500);
        readbuf();
        sendAT(IBEACON, "1");
        delaytick(500);
        readbuf();
        sendAT(IBEACON_MODE, "2");
        delaytick(500);
        readbuf();
        sendAT(POWER_MODE, "0");
        delaytick(500);
        readbuf();
        sendAT(RESET);
        delaytick(500);
        readbuf();
        sendAT(ROLE, "0");
        delaytick(500);
        readbuf();
        sendAT(START);
    }

    int set_beacon_adv(void)
    {
    }

    int set_beacon_scan(void)
    {
    }

    uint32_t gettick(void)
    {
        clock_gettime(CLOCK_MONOTONIC, &ts);
        nowtick = ts.tv_nsec / 1000000;
        nowtick += ts.tv_sec * 1000;
        return nowtick;
    }

    uint32_t delaytick(uint32_t tick)
    {
        nowtick = gettick();
        pasttick = nowtick;
        while (1)
        {
            gettick();
            if (nowtick - pasttick > tick)
            {
                pasttick = nowtick;
                break;
            }
        }
    }

    int readbuf(void)
    {
        read(dev, &buffer, 1000);
        for (int i = 0; i < 1000; i++)
        {
            if (buffer[i] != 0)
            {
                printf("%c, ", buffer[i]);
                buffer[i] = 0;
            }
        }
        printf("\n");
    }

    int run(void)
    {
        nowtick = gettick();

        if (nowtick - pasttick > 2000)
        {
            std::cout << "[nowtick] : " << nowtick << std::endl;

            sendAT(AT);
            delaytick(100);
            readbuf();

            pasttick = nowtick;
        }
    }

    int sendAT(char *input)
    {
        std::cout << "[size] :" << strlen(input) << std::endl;
        // for (int i = 0; i < strlen(input); i++)
        // {
        //     printf("[%d, %c] ", input[i], input[i]);
        // }
        // printf("\n");
        std::cout << "[sendData]" << input << std::endl;
        write(dev, input, strlen(input));
    }

    int sendAT(std::string input)
    {
        int length = input.length();
        char ch[100];
        strcpy(ch, input.c_str());
        std::cout << "[sendData]" << input << std::endl;
        write(dev, ch, length);
    }

    int sendAT(char *input, char *data)
    {
        // std::cout << "[size1] :" << strlen(data) << std::endl;
        // std::cout << "[size2] : " << strlen(input) << std::endl;

        int s1 = strlen(input);
        int s2 = strlen(data);
        int size = strlen(data) + strlen(input);
        char send[size];
        for (int i = 0; i < strlen(input); i++)
        {
            send[i] = input[i];
        }
        for (int i = 0; i < strlen(data); i++)
        {
            send[s1 + i] = data[i];
        }
        for (int i = 0; i < size; i++)
        {
            // printf("[%d, %c] ", send[i], send[i]);
        }
        // printf("\n");
        std::cout << "[sendData]" << send << std::endl;
        // write(dev, input, strlen(input));
        // write(dev, data, strlen(data));
        write(dev, send, size);
    }

    int state_machine(void)
    {
    }

    Euler get_data(void)
    {
        write(dev, Tx, 5);
        read(dev, &buffer, 100);

        if (buffer[0] == 'a' && buffer[1] == 'n' && buffer[2] == 'g')
        {
            char *ptr = strtok(buffer, " ");

            ang_count = 0;

            while (ptr != NULL)
            {
                ang_count++;

                ptr = strtok(NULL, " ");

                if (ang_count == 1)
                {
                    euler.roll = atof(ptr);
                }
                else if (ang_count == 2)
                {
                    euler.pitch = atof(ptr);
                }
                else if (ang_count == 3)
                {
                    euler.yaw = atof(ptr);
                }
            }
        }

        std::cout << "roll = " << euler.roll << std::endl;
        std::cout << "pitch = " << euler.pitch << std::endl;
        std::cout << "yaw = " << euler.yaw << std::endl;
        std::cout << std::endl;

        return euler;
    }

private:
    //
    unsigned long nowtick = 0;
    unsigned long pasttick = 0;
    struct timespec ts;
    // Device Name
    int dev = 0;

    Euler euler;

    // Data buffer
    char buffer[1000];
    unsigned char Tx[5];

    // Serperate Euler Angle Variable
    int ang_count = 0;

    // ASCII CODE
    const unsigned char A = 0x61;
    const unsigned char N = 0x6E;
    const unsigned char G = 0x67;
    const unsigned char CR = 0x0D;
    const unsigned char LF = 0x0A;

    uint8_t *sendData;
    uint8_t *getData;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hm10");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1000);

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("hm10/imu", 1);

    sensor_msgs::Imu msg;

    HM10 hm10;
    Euler euler;

    uint32_t count = 0;

    char mode;
    std::cout << "[MODE] : " << std::endl;
    std::cout << "[u] : user input mode" << std::endl;
    std::cout << "[a] : atomation mode" << std::endl;
    std::cin >> mode;

    while (ros::ok())
    {
        count++;
        std::cout << "[step] : " << count << std::endl;
        if (mode == 'u')
        {
            std::string input;
            std::cout << "[INPUT STRING]" << std::endl;
            std::cin >> input;
            hm10.sendAT(input);
            if (strcmp("AT+DISI?", input.c_str()) == 0)
            {
                std::cout << "Delay more" << std::endl;
                hm10.delaytick(5000);
            }
            else
                hm10.delaytick(1000);
            hm10.readbuf();
        }
        else if (mode == 'a')
        {
            hm10.run();
        }

        msg.orientation.x = euler.roll;
        msg.orientation.y = euler.pitch;
        msg.orientation.z = euler.yaw;

        imu_pub.publish(msg);

        loop_rate.sleep();
    }

    return 0;
}
