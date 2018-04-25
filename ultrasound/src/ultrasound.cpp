/// Ultrasound sensor for Arduino UNO UART interface with ROS
/// Developed by XYLoc.
/// 2018. 04. 06


// essential header for ROS-OpenCV operation
#include <ros/ros.h>

// for using standard messages, float 32 type
// communicate to image processing algorithm result
#include <std_msgs/Float32MultiArray.h>

// for using serial communication
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

#define PORT1 		"/dev/ttyUSB0"
#define BAUDRATE 115200


// setup the initial name
using namespace ros;
using namespace std;


int 	OpenSerial(char *device_name);
int     FdPort1;


// node main loop, for ROS
int main(int argc, char** argv)
{
	// node name initialization
    	init(argc, argv, "ultrasound");

	// assign node handler
	ros::NodeHandle nh_;

	//Publish
        Publisher  P_ULTRASOUND	= nh_.advertise<std_msgs::Float32MultiArray>("/ulstrasound",10);

	// setup the loop speed, [Hz], synchronizing the hector slam loop
        ros::Rate loop_rate(20);
	FdPort1 = OpenSerial(PORT1);

        int i =0;
        // node loop, for ROS, check ros status, ros::ok()
        while( ros::ok() )
	{
    
                char buffer[64];
		std::string response;

                i = 0;

                int n = read((int)FdPort1, buffer, sizeof buffer);

                if (buffer[0] == 'A' && i == 0)
                {
                    response = response + std::string(buffer);
                    i++;
                }
                else
                {}

                if (i>0)
                {
                    cout << response <<"\n";
                    string data1 = response.substr(1,3);
                    string data2 = response.substr(5,3);
                    string data3 = response.substr(9,3);
                    string data4 = response.substr(13,3);


                    int sensor1 = atoi(data1.c_str());
                    int sensor2 = atoi(data2.c_str());
                    int sensor3 = atoi(data3.c_str());
                    int sensor4 = atoi(data4.c_str());

                    cout << "1: " << sensor1 << "\n";
                    cout << "2: " << sensor2 << "\n";
                    cout << "3: " << sensor3 << "\n";
                    cout << "4: " << sensor4 << "\n";

                    // messages
                    std_msgs::Float32MultiArray ultrasound_msg;
                    ultrasound_msg.data.clear();
                    ultrasound_msg.data.resize(4);
                    ultrasound_msg.data[0] = sensor1;
                    ultrasound_msg.data[1] = sensor2;
                    ultrasound_msg.data[2] = sensor3;
                    ultrasound_msg.data[3] = sensor4;
                    P_ULTRASOUND.publish(ultrasound_msg);
                }

		// loop rate [Hz]
        	loop_rate.sleep();

                // loop sampling, ros
		spinOnce();
        }
	return 0;
}


int OpenSerial(char *device_name)
{
	int fd;
	fd = open(device_name, O_RDWR | O_NOCTTY | O_SYNC);
	struct termios newtio;
	if(fd < 0)
	{
		printf("Serial Port Open Fail.\n");
		return -1;
	}

	memset(&newtio, 0, sizeof(newtio));
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_cflag = CS8|CLOCAL|CREAD;

	switch(BAUDRATE)
	{
		case 921600 : newtio.c_cflag |= B921600;
		break;
		case 115200 : newtio.c_cflag |= B115200;
		break;
		case 57600  : newtio.c_cflag |= B57600;
		break;
		case 9600  : newtio.c_cflag |= B9600;
		break;
	}

	newtio.c_lflag 		= 0;
	newtio.c_cc[VTIME] 	= 0;
	newtio.c_cc[VMIN] 	= 17;
	//newtio.c_cc[VMIN] 	= sizeof()/2;

	tcflush(fd,TCIFLUSH);
	tcsetattr(fd,TCSANOW, &newtio);

	return fd;
}
