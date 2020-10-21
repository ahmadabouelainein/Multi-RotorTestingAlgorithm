#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8MultiArray.h"

using namespace std;
	union analogSensorBuffer_T     
	{
	  int analogSensorInt;
	  unsigned char  analogSensorChar [2];
	}analogSensorBuffer;
	
	
int results;
int fd;
int c = 0;
int res;
unsigned char pwm [6] = {0,0,0,0,0,0};

int spiTxRx(unsigned char txDat);
int sendCommand();

void responseCallBack(const std_msgs::UInt8MultiArray::ConstPtr &array){
	int i = 0;
	for(std::vector<uint8_t>::const_iterator it = array->data.begin(); it != array->data.end(); ++it){
		pwm[i] = *it;
		i++;
	}

}
int main (int argc, char **argv){
	fd = open("/dev/spidev0.0", O_RDWR);
	//c = 0;
	ros::init(argc, argv, "spi");
    ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::Int16>("Ultrasonic", 5);
	ros::Rate loop_rate(30);
	ros::spinOnce();
	
	ros::Subscriber sub = n.subscribe("motors",5, responseCallBack);
    ros::spinOnce();
	unsigned int speed = 1000000;
	ioctl (fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	while (ros::ok()){
	
	
		std_msgs::Int16 msg;
		msg.data = sendCommand();
		ROS_INFO("%d", msg.data);
		pub.publish(msg);
		ros::spinOnce();
		
		usleep (30000);
	 }    

}

int spiTxRx(unsigned char txDat)
{
 
  unsigned char rxDat;

  struct spi_ioc_transfer spi;

  memset (&spi, 0, sizeof (spi));

  spi.tx_buf        = (unsigned long)&txDat;
  spi.rx_buf        = (unsigned long)&rxDat;
  spi.len           = 1;

  ioctl (fd, SPI_IOC_MESSAGE(1), &spi);

  return rxDat;
}
int sendCommand(){

	unsigned char Byte;
	bool ack;
	
	
	do
		{
		ack = false;
	
		
		spiTxRx('c');
		usleep (10);
		
		Byte = spiTxRx(0);
		usleep(10);
		if (Byte == 'a')
		{
		ack = true;
		}
		usleep(10);
		}
	while (ack == false);

	Byte = spiTxRx(pwm[0]);
	analogSensorBuffer.analogSensorChar[0] = Byte;
	usleep(10);

	Byte = spiTxRx(pwm[1]);
	analogSensorBuffer.analogSensorChar[1] = Byte;
	usleep(10);
	
	spiTxRx(pwm[2]);
	usleep(10);
	
	spiTxRx(pwm[3]);
	usleep(10);
	
	spiTxRx(pwm[4]);
	usleep(10);
	
	spiTxRx(pwm[5]);
	usleep(10);
	
	return analogSensorBuffer.analogSensorInt;
}
