#include "ros/ros.h"
#include "Serial.h"
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <../include/imu.h>
#include <../include/AT.h>
#include <algorithm>
#include <string>
#include <unistd.h>
#include <time.h>
#include <cmath>

#define BUF_SIZE 2000
#define MODE_BROADCASTING   1
#define MODE_SCAN           2

typedef struct _devices
{
    /* data */
    char factory_id[8];
    char ibeacon_id[32];
    char major[4];
    char minor[4];
    char txpower[2];
    char MAC[12];
    char RSSI[4];
}devices_t;


class HM10
{

public:
    HM10()
    {
        
        for (int i = 0; i < BUF_SIZE; i++)
            buffer[i] = 0;

        dev = open_serial((char *)"/dev/ttyUSB0", 9600, 0, 0);
    }
    HM10(char *_dev)
    {
        std::cout << "dev : " << _dev << std::endl;
        for (int i = 0; i < BUF_SIZE; i++)
            buffer[i] = 0;

        dev = open_serial((char *)_dev, 9600, 0, 0);
    }
    ~HM10()
    {
        close_serial(dev);
    }

    int set_beacon_mode(void)
    {
        delaytick(2000);
        sendAT(AT); delaytick(1000); readbuf();
        sendAT(RESET); delaytick(2000); readbuf();
        sendAT(AT); delaytick(2000); readbuf();
        sendAT(MAJOR, "0x0001"); delaytick(2000); readbuf();
        sendAT(MINOR, "0x0002"); delaytick(1000); readbuf();
        sendAT(ADVERTISING_INTERVAL, "0"); delaytick(1000); readbuf();
        sendAT(NAME, "ACSL"); delaytick(1000); readbuf();
        sendAT(ADVERTISING_TYPE, "3"); delaytick(1000); readbuf();
        sendAT(IBEACON, "1"); delaytick(1000); readbuf();
        sendAT(IBEACON_MODE, "2"); delaytick(2000); readbuf();
        sendAT(POWER_MODE, "1"); delaytick(1000); readbuf();
        sendAT(ROLE, "0"); delaytick(1000); readbuf();
        sendAT(START);
    }

    int set_beacon_adv(void)
    {
        sendAT(ROLE, "0");
        delaytick(1000);
        sendAT(START);
    }

    int set_beacon_scan(void)
    {
        sendAT(ROLE, "1");
        delaytick(1000);
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

    void clearbuf(void)
    {
        
    }
    int readchar(void)
    {

    }
    int readbuf()
    {
        int isdata = 0;
        read(dev, &buffer, BUF_SIZE);
        for (int i = 0; i < BUF_SIZE; i++)
            if(buffer[i] !=0)
            {
                if(buffer[i] > 0)
                {
                    std::cout << buffer[i];
                    isdata = 1;
                }
                state_machine(buffer[i]);
                buffer[i] = 0;
            }

        if(isdata)
            std::cout << std::endl;
    }
    int readbuf2()
    {
        int n;
        // char buffer[1000] = {0,};
        do{
            n = read(dev, &buffer, 1);
            if(n > 0)
            std::cout << (char)n<<","<< n << ",";
        } while(n > 0);
    }
    int readbuf(int size)
    {
        read(dev, &buffer, size);
        for (int i = 0; i < size; i++)
            if(buffer[i] !=0)
            {
                state_machine(buffer[i]);
                buffer[i] = 0;
            }
    }
    int broadcast(void)
    {   
        // sendAT(RESET);
        // delaytick(2000); readbuf();
        sendAT(AT);
        delaytick(100); readbuf();
        sendAT(ROLE, "0");
        delaytick(100); readbuf();
        sendAT(IBEACON_MODE, "2");
        delaytick(2000); readbuf();
        sendAT(START);
        delaytick(100); readbuf();
    }

    int discover(void)
    {
        nowtick = gettick();

        readbuf(100);
        if (nowtick - pasttick > 1000)
        {
            std::cout << "[nowtick] : " << nowtick << std::endl;

            sendAT(IBEACON_DISCOVER, "?");

            pasttick = nowtick;
        }
    }

    int change_mode(void)
    {
        if(mode == MODE_SCAN)
        {
            mode = MODE_BROADCASTING;
            sendAT(ROLE,"0");
            delaytick(100); readbuf();
            sendAT(START);
            delaytick(100); readbuf();
        }
        else if(mode == MODE_BROADCASTING)
        {
            mode = MODE_SCAN;
            sendAT(ROLE,"1");
            delaytick(100); readbuf();
            sendAT(START);
            delaytick(100); readbuf();
            delaytick(100);
        }
    }

    int run(void)
    {
        nowtick = gettick();
        
        if(nowtick - pasttick > 200)
        {
            int rnd = rand() & 100;
            if (rnd > 97)
                change_mode();
            if(mode == MODE_BROADCASTING)
            {            }
            else if(mode == MODE_SCAN)
            {
                readbuf(1000);
                sendAT(IBEACON_DISCOVER, "?");
            }
            pasttick = nowtick;
        }
    }

    int sendAT(char *input)
    {
        //std::cout << "[size] :" << strlen(input) << std::endl;
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
        int s1 = strlen(input);
        int s2 = strlen(data);
        int size = strlen(data) + strlen(input);
        char *a = new char[size];
        char send[size];
        for (int i = 0; i < strlen(input); i++)
            send[i] = input[i];
        for (int i = 0; i < strlen(data); i++)
            send[s1 + i] = data[i];
        std::cout << "[sendData]" << send << "[len, s1, s2]" << size <<", " << s1 <<", "<< s2  << std::endl;
        write(dev, send, size);
    }
    void print(std::string::size_type n, std::string const &s)
    {
        if (n == std::string::npos)
        {
            std::cout << "not found\n";
        }
        else
        {
            std::cout << "found: " << s.substr(n, n+70) << '\n';
        }
    }
    double calculateAccuracy(int txPower, int rssi)
    {
        if(rssi == 0 || txPower == 0)
        {
            return -1;
        }
        
        double ratio = rssi * 1/ txPower;
        if(ratio < 1.0) {
            return pow(ratio, 10);
        }
        else
        {
            return (0.80076) * pow(ratio, 7.7095) +0.111;
        }
        
    }
    int state_machine(char data)
    {
        if(state == 0)
            if(data == 'O')
                state = 1;
            else 
                state = 0;
        else if(state == 1)
            if(data == 'K')
                state = 2;
            else
                state = 0;
        else if(state == 2)
            if(data == '+')
                state = 3;
            else
                state = 0;
        else if(state == 3)
            if(data == 'D')
                state = 4;
            else
                state = 0;
        else if(state == 4)
            if(data == 'I')
                state = 5;
            else
                state = 0;
        else if(state == 5)
            if(data == 'S')
                state = 6;
            else
                state = 0;
        else if(state == 6)
            if(data == 'C')
                state = 7;
            else
                state = 0;
        else if(state == 7)
            if(data == ':')
                state = 8;
            else
                state = 0;
        else if(state >= 8)
        {

            packet[state - 8] = data;
            state++;
            if(state == 70 + 8)
            {
                state = 0;
                char_to_device(packet);
            }
        }
    }
    devices_t char_to_device(char *data)
    {
        std::string str(data);
        devices_t temp;
        strcpy(temp.factory_id, str.substr(0, 8).c_str());
        strcpy(temp.ibeacon_id, str.substr(9, 32).c_str());
        strcpy(temp.major, str.substr(42, 4).c_str());
        strcpy(temp.minor, str.substr(46, 4).c_str());
        strcpy(temp.txpower, str.substr(50, 2).c_str());
        strcpy(temp.MAC, str.substr(53, 12).c_str());
        strcpy(temp.RSSI, str.substr(66, 4).c_str());
    
        if(strcmp(str.substr(0, 8).c_str(), "00000000") == 0)
            return temp;
        if(strcmp(str.substr(9, 32).c_str(), IBEACON_APPLE_LOCATION) != 0)
            return temp;
       
        std::cout << "factory_id : " << temp.factory_id << std::endl;
        std::cout << "ibeacon_id : " << temp.ibeacon_id << std::endl;
        std::cout << "major : " << temp.major << std::endl;
        std::cout << "minor : " << temp.minor << std::endl;
        std::cout << "txpower : " << temp.txpower << std::endl;
        std::cout << "MAC : " << temp.MAC << std::endl;
        std::cout << "RSSI : " << temp.RSSI << std::endl;

        int rssi = atoi(temp.RSSI);
        int txpower = str2hex(temp.txpower[0], temp.txpower[1]);
        std::cout << "rssi:" << rssi << std::endl;
        std::cout << "txpower:" << txpower << std::endl;
        std::cout << "distance:" << calculateAccuracy(txpower, rssi) << std::endl;
        return temp;
    }
    int state_machine(char * data)
    {
        std::cout << "[state machine]" << std::endl;
        //std::cout << data << std::endl;
        
        std::string::size_type n = 0;
        std::string s(data);
        while(1)
        {
            n = s.find("OK+DISC:", n+1);
            if(n == std::string::npos)
            {
                break;
            }
            else
            {
                std::string find = s.substr(n+8,70);
                std::cout << find << std::endl;
                devices_t temp = {0,};
            
                strcpy(temp.factory_id  , find.substr(0,   8).c_str());
                strcpy(temp.ibeacon_id  , find.substr(9,  32).c_str());
                strcpy(temp.major       , find.substr(42,  4).c_str());
                strcpy(temp.minor       , find.substr(46,  4).c_str());
                strcpy(temp.txpower     , find.substr(50,  2).c_str());
                strcpy(temp.MAC         , find.substr(53, 12).c_str());
                strcpy(temp.RSSI        , find.substr(66,  4).c_str());

                std::cout << "factory_id : "    << temp.factory_id  << std::endl;
                std::cout << "ibeacon_id : "    << temp.ibeacon_id  << std::endl;
                std::cout << "major : "         << temp.major       << std::endl;
                std::cout << "minor : "         << temp.minor       << std::endl;
                std::cout << "txpower : "       << temp.txpower     << std::endl;
                std::cout << "MAC : "           << temp.MAC         << std::endl;
                std::cout << "RSSI : "          << temp.RSSI        << std::endl;
                
                int rssi = atoi(temp.RSSI);
                int txpower = str2hex(temp.txpower[0], temp.txpower[1]);
                std::cout << "rssi:" << rssi<< std::endl;
                std::cout << "txpower:" << txpower<< std::endl;
                std::cout << "distance:" <<calculateAccuracy(txpower, rssi) << std::endl;
            }
        }

        std::cout << "[state machine end]" << std::endl;
    }
    int str2hex(char two, char one)
    {
        int t=0;
        int o=0;
        if(two >= 'A' && two <= 'Z')
            t = 16*(two-'A'+10);
        else if(two >= 'a' && two <= 'z')
            t = 16*(two-'a'+10);
        else
            t = 16*(two - '0');
        if(one >= 'A' && one <= 'Z')
            o = (one-'A'+10);
        else if(one >= 'a' && one <= 'z')
            o = (one-'a'+10);
        else
            o = one - '0';

        return t + o;
        
    }
    int find_device(char *id)
    {
        for(int i = 0; i < 100; i ++)
        {
            if(strcmp(devices[i].ibeacon_id, id) == 0)
            {
                return i;
            }
        }
        return -1;
    }
    int getDev(void)
    {
        return dev;
    }

private:
    //
    unsigned long nowtick = 0;
    unsigned long pasttick = 0;
    struct timespec ts;
    // Device Name
    int dev = 0;

    //
    int state = 0;
    char packet[70] = {0,};

    // Data buffer
    char buffer[BUF_SIZE];

    int mode = MODE_BROADCASTING;
    // Devices
    devices_t devices[100];
};




HM10 *hm10;


void sub_at_callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("get AT message : [%s] ", msg->data.c_str());
    hm10->sendAT(msg->data);
 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hm10");
    std::string param1(argv[1]);
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);


    sensor_msgs::Imu msg;
    std::string param;

    char *_dev;
    int len = param.length();
    if (param1.length() > 20)
    {
        nh.getParam("device", param1);
        _dev = new char[len];
        strcpy(_dev, param1.c_str());
    }
    else
    {
        _dev = new char[len];
        strcpy(_dev, param1.c_str());
    }

    hm10 = new HM10(_dev);
    if(hm10->getDev() == -1)
    {
        std::cout << "port error" << std::endl;
        return -1;
    }

    uint32_t count = 0;


    ros::Subscriber sub_at = nh.subscribe("HM10/command",1, sub_at_callback);

    // hm10.set_beacon_mode();

    char mode;
    while (ros::ok())
    {
        std::cout << "[MODE] : " << std::endl;
        std::cout << "[u] : user input mode" << std::endl;
        std::cout << "[a] : atomation  edit(hm.run())" << std::endl;
        std::cout << "[d] : discovery mode (discover Air location)" << std::endl;
        std::cout << "[b] : broadcast mode (broadcast Air location)" << std::endl;
        std::cout << "[s] : set beacon mode" << std::endl;
        std::cout << "[r] : set ros AT mode" << std::endl;
        std::cin >> mode;
        if(mode == 's')
            hm10->set_beacon_mode();
        else
            break;
        
    }

    while (ros::ok())
    {
        count++;
        // std::cout << "[step] : " << count <<"mode:"<< mode << std::endl;
        if (mode == 'u')
        {
            std::string input;
            std::cout << "[INPUT STRING]" << std::endl;
            std::cin >> input;
            hm10->sendAT(input);
            if (strcmp("AT+DISI?", input.c_str()) == 0)
            {
                //std::cout << "Delay more" << std::endl;
                //hm10.delaytick(5000);
                hm10->delaytick(1000);
                hm10->readbuf();

            }
            else
            {
                hm10->delaytick(1000);
                hm10->readbuf();
            }
        }
        else if (mode == 'a')
        {
            hm10->run();
        }
        else if (mode == 'd')
        {
            hm10->discover();
        }
        else if (mode == 'b')
        {
            hm10->broadcast();
            mode = 'w';
        }
        else if (mode == 'r')
        {
            hm10->readbuf();
            ros::spinOnce();
        }
        else if (mode == 'w')
        {

        }

        loop_rate.sleep();
        
    }

    return 0;
}
