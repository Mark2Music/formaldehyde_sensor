#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h> 
#include <stdio.h>
#include <stdlib.h>
#include <sys/select.h>

class SerialPort
{
    public :
		// set neccessary params  
		bool Set_Parity(int fd,int databits,int stopbits,int parity);	
		// set the speed of the serial port
		void Set_Speed(int fd, int speed);
		//receive the cmd and deal it
		char* itoa(int num, char*str, int radix);
        //create two static variables   
		static int speed_arr[];	
		static int name_arr[];
		//some useful variables
		bool already_open_flag;
		int fd;
		char send_buffer[100];//连接设备时，每个步骤分配三个字节
		int count;
		bool blueteeth_connect_status; // true is connected, false is unconnected
};

SerialPort serialport;
fd_set rfds;

int SerialPort::speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300};
int SerialPort::name_arr[] = {115200, 38400, 19200, 9600, 4800, 2400, 1200, 300};

char* SerialPort::itoa(int num, char* str, int radix)
{
	char index[]="0123456789ABCDEF";
	unsigned unum;/*中间变量*/
	int i=0,j,k;
	/*确定unum的值*/
	if(radix==10 && num<0)/*十进制负数*/
	{
		unum=(unsigned)-num;
		str[i++]='-';
	}
	else 
		unum=(unsigned)num;/*其他情况*/
		
	do
	{
		str[i++]=index[unum%(unsigned)radix];
		unum/=radix;
	}while(unum);
	
	str[i]='\0';
	if(str[0]=='-')
		k=1;/*十进制负数*/
	else
		k=0;
	char temp;
	for(j=k;j<=(i-1)/2;j++)
	{
		temp=str[j];
		str[j]=str[i-1+k-j];
		str[i-1+k-j]=temp;
	}
	return str;
}

bool SerialPort::Set_Parity(int fd,int databits,int stopbits,int parity)
{
    struct termios options;   
    if  ( tcgetattr( fd, &options) !=  0) 
    {   
        perror("SetupSerial failed!");       
        return false;    
    }  
    options.c_cflag &= ~CSIZE;   
    switch (databits)   
    {     
        case 7:       
            options.c_cflag |= CS7;   
            break;  
        case 8:       
            options.c_cflag |= CS8;  
            break;     
        default:      
            fprintf(stderr,"Unsupported data size\n"); 
            return false;    
    }  
    switch (parity)   
    {     
        case 'n':  
        case 'N':      
            options.c_cflag &= ~PARENB;   /* Clear parity enable */ 
            options.c_iflag &= ~INPCK;     /* Enable parity checking */   
            break;    
        case 'o':     
        case 'O':       
            options.c_cflag |= (PARODD | PARENB);   
            options.c_iflag |= INPCK;             /* Disnable parity checking */   
            break;    
        case 'e':    
        case 'E':     
            options.c_cflag |= PARENB;     /* Enable parity */      
            options.c_cflag &= ~PARODD;      
            options.c_iflag |= INPCK;       /* Disnable parity checking */  
            break;  
        case 'S':   
        case 's':  /*as no parity*/     
            options.c_cflag &= ~PARENB;  
            options.c_cflag &= ~CSTOPB;
            break;    
        default:     
            fprintf(stderr,"Unsupported parity\n");      
            return false;    
    }    
      
    switch (stopbits)  
    {     
        case 1:      
            options.c_cflag &= ~CSTOPB;    
            break;    
        case 2:      
            options.c_cflag |= CSTOPB;    
           break;  
        default:      
            fprintf(stderr,"Unsupported stop bits\n");    
            return false;   
    }   
    /* Set input parity option */   
    if (parity != 'n')     
        options.c_iflag |= INPCK;   

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &=~(ICANON | ECHO | ECHOE | ISIG);
 
    options.c_oflag &=~OPOST;
 
    options.c_iflag &=~ICRNL;
    options.c_iflag &=~INLCR;
	
    //tcflush(fd,TCIFLUSH);
    tcflush(fd,TCIOFLUSH);
    //options.c_cc[VTIME] = 150;   
    //options.c_cc[VMIN] = 0; /* Update the options and do it NOW */  
    /* 这样设置表示read函数只有在读取一个字节后才能返回
     * 否则一直等待, 上面注释里面的方式是正确的
     */
    options.c_cc[VTIME] = 100;
    options.c_cc[VMIN] = 9;
    if (tcsetattr(fd,TCSANOW,&options) != 0)     
    {   
        perror("SetupSerial failed!\n");     
        return false;    
    }   
    return true;
}

void SerialPort::Set_Speed(int fd, int speed)
{  
  int   i;   
  int   status;   
  struct termios Opt;
  
  tcgetattr(fd, &Opt);   
  for ( i= 0;  i < sizeof(serialport.speed_arr) / sizeof(int);  i++) 
  {   
    if(speed == serialport.name_arr[i]) 
	{       
      tcflush(fd, TCIOFLUSH);       
      cfsetispeed(&Opt, serialport.speed_arr[i]);    
      cfsetospeed(&Opt, serialport.name_arr[i]);     
      status = tcsetattr(fd, TCSANOW, &Opt);    
      if  (status != 0) 
	  	{          
          perror("tcsetattr error");    
          return;       
        }      
      tcflush(fd, TCIOFLUSH);
	  printf(" set serial 115200...\n");
	  
	  return;
    }    
  } 
  
}

int main(int argc, char **argv)
{
  
	ros::init(argc, argv, "formaldehyde_sensor");
	ros::NodeHandle n;
	
	serialport.already_open_flag = false;
	serialport.fd = -1;
	serialport.send_buffer[100] = {0};
	serialport.send_buffer[0] = -1;
	serialport.blueteeth_connect_status = false;
	
	//serialport initialize
	//need have a special device name according the ID
	serialport.fd = open("/dev/fhsensorserial0", O_RDWR);  
	if(serialport.fd == -1)  
	{  
		perror("set serialport error\n");  
	}  
	else  
	{  
		printf("%s",ttyname(serialport.fd));  
		printf(" succesfully\n");  
	}  
	//set the parameters of serialport  
	serialport.Set_Speed(serialport.fd, 9600);
	if (serialport.Set_Parity(serialport.fd, 8, 1, 'N') == true)  
	{  
		printf("set data format succesfully!\n");  
	}
	else
	{
		printf("set data format failed!\n");
        exit(-1);
	}
	
	/*将文件描述符加入读描述符集合*/
	FD_ZERO(&rfds);
	FD_SET(serialport.fd, &rfds);
	ros::Publisher sensor_pub = n.advertise<std_msgs::Int16MultiArray>("sensor_value", 1000);
	
	ros::Rate loop_rate(0.8);
	
	while (ros::ok())
	{	
	        std_msgs::Int16MultiArray msg;
                struct timeval time;
                time.tv_sec = 1;
		time.tv_usec = 0;
		int ret = select(serialport.fd+1, &rfds, NULL, NULL, &time);
			
		if (ret < 0)
		{
		    perror("select");
		    continue;
		}
		else if ( ret == 0 )
		{
		    std::cout<<"timeout!\n"<<std::endl;
		    continue;
		}
		else
		{
			//printf("......get sensor value......");
			//std::cout<<std::endl;
			ret = read(serialport.fd, &serialport.send_buffer[0], 9);//只是返回"OK"
			//print the string
			//printf("ret = %d, %0x %0x\n", ret, serialport.send_buffer[2], serialport.send_buffer[3]);
			//std::cout<<std::endl;

                        //clear uart data.
        		tcflush(serialport.fd, TCIOFLUSH);
                        
                        msg.data.push_back(serialport.send_buffer[2]*256+serialport.send_buffer[3]);//df
                        msg.data.push_back(serialport.send_buffer[4]*256+serialport.send_buffer[5]) ;//temperature
                        msg.data.push_back(serialport.send_buffer[6]*256+serialport.send_buffer[7]);//dampness
                        //printf("df = %d, tem = %d, damp = %d \n", msg.data[0], msg.data[1], msg.data[2]);
                        //publish the sensor data
                        sensor_pub.publish(msg);
			//destroy the data, avoid cost so much memory
			while (!msg.data.empty())
  			{
    				msg.data.pop_back();//删除最后一个位置的元素
  			}
			
		}
		
                
		ros::spinOnce();

		loop_rate.sleep();
	}
	
	close(serialport.fd);

	return 0;
}
