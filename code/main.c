#include<arpa/inet.h>
#include<sys/types.h>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<unistd.h>
#include<sys/socket.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<sys/ioctl.h>
#include<netinet/in.h>
#include<pthread.h>
#include<linux/input.h>
#include<sys/mman.h>
#include<ctype.h>
#include<termios.h>
#include<math.h>

#define PORT_s	5011
#define PORT_c	5010

#define TEXTLCD_BASE 0xbc
#define TEXTLCD_COMMAND_SET _IOW(TEXTLCD_BASE, 0, int)
#define TEXTLCD_FUNCTION_SET _IOW(TEXTLCD_BASE, 1, int)
#define TEXTLCD_DISPLAY_CONTROL _IOW(TEXTLCD_BASE, 2, int)
#define TEXTLCD_CURSOR_SHIFT _IOW(TEXTLCD_BASE, 3, int)
#define TEXTLCD_ENTRY_MODE_SET _IOW(TEXTLCD_BASE, 4, int)
#define TEXTLCD_RETURN_HOME _IOW(TEXTLCD_BASE, 5, int)
#define TEXTLCD_CLEAR _IOW(TEXTLCD_BASE, 6, int)
#define TEXTLCD_DD_ADDRESS _IOW(TEXTLCD_BASE, 7, int)
#define TEXTLCD_WRITE_BYTE _IOW(TEXTLCD_BASE, 8, int)

#define FULL_LED1 9
#define FULL_LED2 8
#define FULL_LED3 7
#define FULL_LED4 6

#define PROXIMITYIO 0x51
#define PROXIMITY_IOCTL_ENABLE _IO(PROXIMITYIO, 0x31)
#define PROXIMITY_IOCTL_DISABLE _IO(PROXIMITYIO, 0x32)
#define PROXIMITY_IOCTL_IS_ENABLE _IOR(PROXIMITYIO, 0x33, int)
#define PROXIMITY_IOCTL_DELAY_GET _IOR(PROXIMITYIO, 0x34, int)
#define PROXIMITY_IOCTL_DELAY_SET _IOR(PROXIMITYIO, 0x35, int)
#define PROXIMITY_IOCTL_DATA _IOR(PROXIMITYIO, 0x36, int)

#define IM3640IO 0x50
#define IM3640_IOCTL_ENABLE _IO(IM3640IO, 0x31)
#define IM3640_IOCTL_DISABLE _IO(IM3640IO, 0x32)
#define IM3640_IOCTL_IS_ENABLE _IOR(IM3640IO, 0x33, int)
#define IM3640_IOCTL_DELAY_GET _IOR(IM3640IO, 0x34, int)
#define IM3640IO_IOCTL_DELAY_SET _IOR(IM3640IO, 0x35, int)
#define IM3640_IOCTL_DATA _IOR(IM3640IO, 0x36, int[9])

#define ACCELATION(x) ((((9.8*2)/256)/256)*x)
#define MEGNETIC(x) (0.2*x)
#define ORIENTATION(x) (0.1*x)

#define FPGA_BASEADDRESS 0x05000000
#define LED_OFFSET 0x20

#define rad(x) x*3.14159??80.0

#define EVENT_BUF_NUM 64

struct strcommand_varible {
        char rows;
        char nfonts;
        char display_enable;
        char cursor_enable;
        char nblink;
        char set_screen;
        char set_rightshit;
        char increase;
        char nshift;
        char pos;
        char command;
        char strlength;
        char buf[16];
};

/*
typedef struct tagPT
{
        double x;
        double y;
        double z;
};
*/
char buf[256];
static char *b;

//value for sending at client
static float aileron=0.0, elevator=0.0;
static double l_throttle=0, r_throttle=0;
static float rudder=0.0;
static int brake=1, fuel_dump=0, view=0;

static float d_la=0.0, d_lo=0.0;
static float remain=0;
char destination[5];

static struct termios initial_settings, new_settings;
static int peek_character=-1;
int data=0;

static float fuel, la_deg, lo_deg, altitude, speed;

int mode=0, auto_s=0;
static int des=0;
/*
tagPT pt1;
tagPT pt2;
pt1.y=36.4;
pt1.x=127.4;
pt2.y=36.4;
pt2.y=127.4;
*/

//double GetDistance( tagPT pt1, tagPT pt2 );

void *th_impr(void *vargp);
void *thread_keypad(void *vargp);

void *th_server(void *vargp);
void *thread_tlcd(void *vargp);
void *th_led(void *vargp);
void *th_7seg(void *vargp);
void *thread_piezo(void *vargp);
void *thread_fled(void *vargp);

void *th_auto(void *vargp);
void *th_start(void *vargp);

int main(void){//client
    	int sd,n;
    	char buf[256];
    	struct sockaddr_in sin;
	pthread_t tid[10];


	pthread_create(&tid[0], NULL, th_impr, NULL);
	pthread_create(&tid[1], NULL, thread_keypad, NULL);

	pthread_create(&tid[2], NULL, th_server, NULL);
	pthread_create(&tid[3], NULL, thread_tlcd, NULL);
	pthread_create(&tid[4], NULL, th_led, NULL);
        pthread_create(&tid[5], NULL, th_7seg, NULL);
        pthread_create(&tid[6], NULL, thread_piezo, NULL);
        pthread_create(&tid[7], NULL, thread_fled, NULL);

        pthread_create(&tid[8], NULL, th_start, NULL);
        pthread_create(&tid[9], NULL, th_auto, NULL);

	printf("\n Hello! Thank you for using our program\n");
	printf(" Here are some information for flying\n\n");

	printf(" Initialize aileron&elevator : get close to proximity sensor\n");
	printf(" Handle aileron&elevator : move board\n");
	printf("****These two option will not apply when you set auto mode**** \n\n");

	printf("          Textlcd\n");
	printf(" line 1 : destination latitude/longtitude\n");
	printf(" line 2 : your latitude/longtitude\n\n");

	printf(" led : speed of filght\n\n");

	printf(" 7-segment : remain distance\n\n");

	printf(" piezo : altitude (enough altitude-stop)\n\n");

	printf(" full-color-led : fuel (green : enough, red : shortage)\n\n");

        printf("                        keypad inform\n");
        printf("  1 : view        2 : brake       3 : elevator-  4 : elevator+\n");
        printf("  5 : l_throttle  6 : r_throttle  7 : aileron-   8 : aileron+ \n");
        printf("  9 : rudder--   10 : rudder-    11 : rudder+   12 : rudder++ \n");
        printf(" 13 : des_1      14 : des_2      15 : des_3     16 : auto/user mode\n");
        printf("\n");

	printf("Let's start!\n\n");

	usleep(500000);


	while(1){
        usleep(100000);
	sd=socket(AF_INET,SOCK_DGRAM,0);

        memset(&sin,0,sizeof(sin));
        sin.sin_family=AF_INET;
        sin.sin_port=htons(PORT_c);
        sin.sin_addr.s_addr=inet_addr("210.117.181.108");
        sprintf(buf, "%f\t%f\t%f\t%f\t%f\t%d\t%d\t%d\t%d\n", aileron, elevator, l_throttle, r_throttle, rudder, brake, fuel_dump, view);
        sendto(sd,buf,strlen(buf),0,(struct sockaddr *)&sin,sizeof(sin));
//send
	}

        return 0;
}

//thread thar use sensor (treate aileron/elevator)
void *th_impr(void *vargp){
	int fi=-1, fp=0;
	int i, j;
	int ri, rp;
        int recv_buf[9]={0};
        float data_buf[9]={0};


	if((fi=open("/dev/im3640", O_WRONLY))<0){
                printf("application : /dev/im3640 open fail!\n");
                exit(1);
        }

	fp=open("/dev/proximity", O_RDONLY);
        if(fp<0){
                perror("/dev/proximity");
                exit(1);
        }


	ri=ioctl(fi, IM3640_IOCTL_ENABLE);
        rp=ioctl(fp, PROXIMITY_IOCTL_ENABLE);

	usleep(200000);
	while(1){
		if(mode==0|auto_s==0){
			ri=ioctl(fi, IM3640_IOCTL_DATA, recv_buf);
                	for(j=0; j<9; j++){//save im sensor data
                	        if(j<3)
                	                data_buf[j]=ACCELATION(recv_buf[j]);
                	        else if(j<6)
                	                data_buf[j]=MEGNETIC(recv_buf[j]);
                	        else
	                                data_buf[j]=ORIENTATION(recv_buf[j]);
	                }/*
	                printf("accel_x=%f, accel_y=%f, accel_z=%f\n", data_buf[0], data_buf[1], data_buf[2]);
	                usleep(500);
	                printf("compass_x=%f, compass_y=%f, compass_z=%f\n", data_buf[3], data_buf[4], data_buf[5]);
	                usleep(500);
	                printf("ori_x=%f, ori_y=%f, ori_z=%f\n", data_buf[6], data_buf[7], data_buf[8]);
	                usleep(500);
	                printf("\n");*/

			read(fp, &data, sizeof(int));//save proximity sensor data

			if(data>2000){//if close
				printf("initialize aileron&elevator\n");
				aileron=0.0;
				elevator=0.0;//initialize
			}
			else{
				//treate elevator value
		                if(data_buf[2]>0.0025&&data_buf[2]<0.0125)
	        	                elevator=-0.75;
	        	        else if(data_buf[2]>0.0125&&data_buf[2]<0.0225)
	        	                elevator=-0.5;
	        	        else if(data_buf[2]>0.0225&&data_buf[2]<0.0325)
	        	                elevator=-0.25;
	        	        else if(data_buf[2]>0.0325&&data_buf[2]<0.0425)
	        	                elevator=0;
	                	else if(data_buf[2]>0.0425&&data_buf[2]<0.0525)
	        	                elevator=0.25;
	                	else if(data_buf[2]>0.0525&&data_buf[2]<0.0625)
	        	                elevator=0.5;
	                	else if(data_buf[2]>0.0625&&data_buf[2]<0.0725)
	        	                elevator=0.75;
	                	else
	        	                elevator=0;

				//treate aileron value
		                if(data_buf[0]>-0.075&&data_buf[0]<-0.055)
					aileron=-0.75;
		                else if(data_buf[0]>-0.055&&data_buf[0]<-0.035)
					aileron=-0.5;
		                else if(data_buf[0]>-0.035&&data_buf[0]<-0.015)
					aileron=-0.25;
		                else if(data_buf[0]>-0.015&&data_buf[0]<0.015)
					aileron=0;
		                else if(data_buf[0]>0.015&&data_buf[0]<0.035)
					aileron=0.25;
		                else if(data_buf[0]>0.035&&data_buf[0]<0.055)
					aileron=0.5;
		                else if(data_buf[0]>0.055&&data_buf[0]<0.075)
					aileron=0.75;
				else
					aileron=0;


			}
		usleep(3000000);
		}
	}

	return NULL;
}

//keypad thread
void *thread_keypad(void *vargp){
	int i;
	int fd=-1;
	int start_b=0;
	size_t read_bytes;
	struct input_event event_buf[EVENT_BUF_NUM];

	if((fd=open("/dev/input/event2", O_RDONLY))<0){
		printf("application : keypad driver open fail!\n");
		exit(1);
	}

	//printf("keypad thread\n");
	while(1){
		usleep(5000);
		read_bytes=read(fd, event_buf, (sizeof(struct input_event)*EVENT_BUF_NUM));
		if(read_bytes<sizeof(struct input_event)){
			printf("applicarion : read error!!\n");
			exit(1);
		}
		for(i=0; i<(read_bytes/sizeof(struct input_event)); i++){
			if((event_buf[i].type==EV_KEY)&&(event_buf[i].value==0)){
				printf(" Button key : %d \t", event_buf[i].code);
				switch(event_buf[i].code){
                                        case 1://change view
						if(view==7) view=0;
						else view=view+1;
						printf("view : %d\n", view);
						break;

                                        case 2://brake on/off
						printf("brake ");
						if(brake==1){ brake=0; printf("off\n");}
                                                else{ brake=1; printf("on\n");}
                                                break;
                                        case 3://treate elevator
						if(elevator>-1.0)
                                                	elevator-=0.025;
						printf("elevator : %.3f\n", elevator);
                                                break;
                                        case 4:
						if(elevator<1.0)
                                                	elevator+=0.025;
						printf("elevator : %.3f\n", elevator);
                                                break;
                                        case 5://left throttle on
						if(l_throttle==0) l_throttle=1.0;
						else l_throttle==0;
						printf("l_throttle\n");
                                                break;
                                        case 6://right throttle on
						if(r_throttle==0) r_throttle=1.0;
						else r_throttle==0;
						printf("r_throttle\n");
                                                break;
                                        case 7://treate aileron
						if(aileron>-1.0)
                                                	aileron-=0.025;
						printf("aileron : %.3f\n", aileron);
                                                break;
                                        case 8:
						if(aileron<1.0)
                                                	aileron+=0.025;
						printf("aileron : %.3f\n", aileron);
                                                break;
                                        case 9://treate rudder
                                                if(rudder>-1.0)
							rudder-=0.2;
						printf("rudder : %.2f\n", rudder);
                                                break;
                                        case 10:
                                                if(rudder>-1.0)
                                                	rudder-=0.1;
						printf("rudder : %.2f\n", rudder);
                                                break;
                                        case 11:
                                                if(rudder<1.0)
                                                	rudder+=0.1;
						printf("rudder : %.2f\n", rudder);
                                                break;
                                        case 12:
                                                if(rudder<1.0)
                                                	rudder+=0.2;
						printf("rudder : %.2f\n", rudder);
                                                break;
                                        case 13://set destination 1
                                                des=1;
						d_la=100.0;
						d_lo=100.0;
						printf("des : %d, la : %.1f lo : %.1f\n", des, d_la, d_lo);
                                                break;
                                        case 14://set destination 2
                                                des=2;
						d_la=200.0;
						d_lo=200.0;
						printf("des : %d, la : %.1f lo : %.1f\n", des, d_la, d_lo);
                                                break;
					case 15://set auto start
						if(auto_s==0){
							auto_s=1;
							printf("%d ", auto_s);
							printf("auto start\n");
						}
						else{
							auto_s=0;
							printf("%d ", auto_s);
							printf("auto start reset\n");
						}
						break;
					case 16://set auto mode
						if(mode==1){ mode=0; printf("user ");}
						else{ mode=1; printf("auto ");}
						printf("mode\n");
						break;
					dafault:
						break;
				}
			}
		}
		usleep(15000);
	}
	close(fd);

	return NULL;
}


void *th_server(void *vargp){
        int i;
        struct sockaddr_in sin, cli;
        int sd,clientLen=sizeof(cli);
        float tmp;

        usleep(20000);

        if((sd=socket(AF_INET,SOCK_DGRAM,0))==-1){
                perror("sd");
                exit(1);
        }

        memset(&sin,0,sizeof(sin));
        sin.sin_family=AF_INET;
        sin.sin_port=htons(PORT_s);
        sin.sin_addr.s_addr=inet_addr("0.0.0.0");
        if( bind(sd,(struct sockaddr *)&sin,sizeof(sin)) ){
                perror("bind");
                exit(1);
        }

	//printf("server thread\n");
        while(1){
                usleep(8000);
		//recieve data
                if(recvfrom(sd, buf, 255, 0, (struct sockaddr *)&cli, &clientLen)==-1){
                        perror("recvfrom");
                        exit(1);
                }

		//separate buf, and chage type and save ar valuable
                b=strtok(buf, "\t");
                tmp=atof(b);
                fuel=tmp;

                b=strtok(NULL, "\t");
                tmp=atof(b);
                la_deg=tmp;

                b=strtok(NULL, "\t");
                tmp=atof(b);
                lo_deg=tmp;

                b=strtok(NULL, "\t");
                tmp=atof(b);
                altitude=tmp;

                b=strtok(NULL, "\t");
                tmp=atof(b);
                speed=tmp;

		//printf("fuel : %f\n", fuel);

                usleep(12000);

	}


	return NULL;
}

void *thread_tlcd(void *vargp){
        int i, dev;
	float a_deg=100;
        char buf0[16] = "                ";
        char buf1[16] = "                ";

        struct strcommand_varible strcommand;
        strcommand.rows = 0;
        strcommand.nfonts = 0;
        strcommand.display_enable = 1;
        strcommand.cursor_enable = 0;
        strcommand.nblink = 0;
        strcommand.set_screen = 0;
        strcommand.set_rightshit = 1;
        strcommand.increase = 1;
        strcommand.nshift = 0;
        strcommand.pos = 10;
        strcommand.command = 1;
        strcommand.strlength = 16;
        dev = open("/dev/textlcd",O_WRONLY|O_NDELAY);
        if (dev != -1){
		//printf("textlcd thread\n");
                while(1){
			usleep(14000);

			//write data buf0, buf1
                        sprintf(buf0, "%.1f %.1f\0", d_la, d_lo);
			sprintf(buf1, "%.1f %.1f\0", la_deg, lo_deg);

			//print ar txtlcd
                        strcommand.pos = 0;
                        ioctl(dev,TEXTLCD_DD_ADDRESS,&strcommand,32);
                        write(dev,buf0,16);
                        strcommand.pos = 40;
                        ioctl(dev,TEXTLCD_DD_ADDRESS,&strcommand,32);
                        write(dev,buf1,16);
			usleep(6000);
                }
                close(dev);
        }
        else
        {
                printf( "\n%d  application : Device Open ERROR!\n",dev);
                exit(1);
        }

        return NULL;
}

void *th_led(void *vargp){	
        int fd, led_val=0x01, val=0x01, tp, i;
        unsigned short *addr_fpga, *addr_led;

        if((fd=open("/dev/mem", O_RDWR|O_SYNC))<0){
                perror("mem open fail\n");
                exit(1);
        }

        addr_fpga=(unsigned short*)mmap(NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, fd, FPGA_BASEADDRESS);

        if(*addr_fpga==(unsigned short)-1){
                close(fd);
                printf("mmap error\n");
                exit(1);
        }

        addr_led=addr_fpga+LED_OFFSET/sizeof(unsigned short);
        *addr_led=0x100;

        //printf("led thread\n");
        while(1){
                usleep(13500);
                tp=(int)speed;
                //printf("tp : %d\t", tp);
                led_val=0x01;
                val=0x01;
		//through speed
                for(i=0; i<(tp/100); i++){
                        val=val<<1;
                        led_val=val+led_val;//calculate led_val
			//printf("speed : %d\n", tp);
                }
                //printf("led_val : 0x%d%d\n", led_val/16, led_val%16);
                *addr_led=led_val|0x100;
                usleep(6500);
        }

        *addr_led=0x00;
        munmap(addr_fpga, 4096);
        close(fd);

	return NULL;
}

void *th_7seg(void *vargp){
        int fd, i, v;
	float tmp;
        if((fd=open("/dev/segment", O_RDWR | O_SYNC))<0){
                printf("FND open fail\n");
                exit(1);
        }

	tmp=(((la_deg-d_la)*(la_deg-d_la))+((lo_deg-d_lo)*(lo_deg-d_lo)));
	v=(int)tmp;//calculate remain destination
	//printf("7segment thread\n");
	usleep(100000);
        while(1){
                for(i=0; i<134; i++){
                        write(fd, &v, 4);
			//calculate remain value
			tmp=(((la_deg-d_la)*(la_deg-d_la))+((lo_deg-d_lo)*(lo_deg-d_lo)));
			remain=tmp;
                        usleep(1000);
			v=(int)tmp;
			if(remain>tmp+100){//if remain value goin too big -> yourgo wron wat
				printf("wrong way!!\n");
			}
                }
        }
        close(fd);

        return NULL;


}

void *thread_fled(void *varp){
	int fd;
	char v[3]={0, 100, 0};

	if((fd=open("/dev/fullcolorled", O_RDWR|O_SYNC))<0){
		perror("/dev/fullcolorled open fail\n");
		exit(1);
	}

	ioctl(fd, FULL_LED1);
        write(fd, v, 3);
        ioctl(fd, FULL_LED2);
        write(fd, v, 3);
        ioctl(fd, FULL_LED3);
        write(fd, v, 3);
        ioctl(fd, FULL_LED4);
        write(fd, v, 3);//initialize to all green

	//printf("f-led thread\n");
	while(1){
		usleep(12500);
		v[1]=100;
		v[0]=0;
		ioctl(fd, FULL_LED1);
        	write(fd, v, 3);
        	ioctl(fd, FULL_LED2);
        	write(fd, v, 3);
        	ioctl(fd, FULL_LED3);
        	write(fd, v, 3);
        	ioctl(fd, FULL_LED4);
        	write(fd, v, 3);
		if(fuel<1500){
			v[1]=0;
			v[0]=100;
			ioctl(fd, FULL_LED1);
			write(fd, v, 3);//lower than 1500, red : 1
                	if(fuel<1050){
                        	ioctl(fd, FULL_LED2);
                        	write(fd, v, 3);//lower than 1050, red 2
                		if(fuel<700){
                        		ioctl(fd, FULL_LED3);
                        		write(fd, v, 3);//lower than 700, red 3
                			if(fuel<350){
                        			ioctl(fd, FULL_LED4);
                        			write(fd, v, 3);//lower than 350, all red
					}
				}
			}
		}
		usleep(7500);
	}
	close(fd);

	return NULL;
}

void *thread_piezo(void *vargp){
        int fd, i, v=0;
        if((fd=open("/dev/piezo", O_RDWR|O_SYNC))<0){
                perror("device open fail\n");
                exit(1);
        }
        //printf("piezo thread\n");
        while(1){
                usleep(12000);
		//printf("altitude : %f\n", altitude);
                if(altitude>1000)
                        write(fd, &v, 4);
                if(altitude>1000&&altitude<1500){
                        v=1;
                        write(fd, &v, 4);
                        sleep(1);
                        v=0;
                        write(fd, &v, 4);
                }
		//if it go higher, print higher sound
                else if(altitude>1500&&altitude<5500){
                        v=(altitude+500)/1000;
                        write(fd, &v, 4);
                        sleep(1);
                        v=0;
                        write(fd, &v, 4);
                }
                else{//high enough, no sound
                        v=0;
                        write(fd, &v, 4);
                }
                usleep(8000);
        }
        close(fd);

        return NULL;
}

//auto mode
void *th_auto(void *vargp){
	sleep(1);
	//printf("auto thread\n");
	while(1){
		if(mode==1){//if user set auto mode
			if(aileron>0.2|aileron<-0.2){//aileron value over
				printf("init aileron - %.2f to ", aileron);
				aileron=0.0;//initialize
				usleep(1000000);
				printf("%.2f\n", aileron);
			}

			if(altitude>9000){//if hign
				elevator=0.03;//go down
/*
				if(elevator<0.2)
					elevator+=0.009;
				else
					elevator=0.09;*/
				printf("%.1f altitude high go down %.3f\n", altitude, elevator);

				sleep(5);
				elevator=0.0;//initialize not to fall
				sleep(2);
			}
			else if(altitude<8500){//if low
				elevator=-0.03;//fo up
				/*
				if(elevator>-0.4)
					elevator-=0.06;
				else
					elevator=-0.4;*/
				printf("%.1f altitude low go up %.3f\n", altitude, elevator);

				sleep(5);
				elevator=0.0;//initialize not to fall
				sleep(2);
			}
			else{
				elevator=0.0;//else go ahead
			}
		}
	}

	return NULL;
}

//auto start
void *th_start(void *vargp){
	while(1){
		if(auto_s==1){//if user set auto start
			if(altitude<3000){
				printf("elevator : %.3f\n", elevator);
				l_throttle=1.0;
				r_throttle=1.0;//throttle on
				usleep(300000);
				brake=0;//brake off
				usleep(100000);
				aileron=0.0;
				elevator=-0.04;//set aileron/elevator
				printf("alti : %.1f ele : %.3f\n", altitude, elevator);
			}
			else{
				elevator=0.0;
			}
		}
	}
	return NULL;
}

/*
double GetDistance(tagPT pt1, tagPR pt2){
        int radius = 6371;

        double dLat = rad( (pt2.y-pt1.y) );
        double dLon = rad( (pt2.x-pt1.x) );

        pt1.y = rad( pt1.y );
        pt2.y = rad( pt2.y );

        double a = sin(dLat??) * sin(dLat??) + sin(dLon??) * sin(dLon??) * cos(pt1.y) * cos(pt2.y);
        double c = 2 * atan2f(sqrtf(a), sqrtf(1-a));
        double dDistance = radius * c;

        dDistance*=1000;

        return dDistance;
}
*/
