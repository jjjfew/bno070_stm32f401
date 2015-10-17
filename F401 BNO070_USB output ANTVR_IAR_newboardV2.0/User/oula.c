#include "stm32f4xx_hal.h"
#include "cmsis_os.h"  
/* Private variables ---------------------------------------------------------*/
#include "stm32f4xx_hal_uart.h"
/* USER CODE BEGIN 0 */
#include "bno070.h"
#include "usb_device.h"
#include <math.h>
#include "oula.h"
#include "usbd_hid.h"

extern UART_HandleTypeDef huart2;
float last_val;
float last_val2;
int8_t send_buf[16]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

void float_char(float f,unsigned char *s)
{
 unsigned char *p;
 
 p = (unsigned char *)&f;

    *s = *p;

    *(s+1) = *(p+1);

    *(s+2) = *(p+2);

    *(s+3) = *(p+3);
}

//*********************************************
//		四元素转欧拉角（单位：弧度）           注意坐标轴方向
//*********************************************
void Quat2Euler(float *q, float *e)       //*q为四元数，*e为欧拉角
{
    float Q[4];
    float T[3][3];
    float Pitch=e[0];  //x
    float Yaw=e[1];     //y
    float Roll=e[2];    //z
      
	Q[0] = q[3];       //w
	Q[1] = q[0]*(-1);  //x 
	Q[2] = q[1]*(-1);  //y
	Q[3] = q[2]*(-1);  //z
    T[0][0] =   Q[0]*Q[0]+(-Q[1])*(-Q[1])-(-Q[2])*(-Q[2])-(-Q[3])*(-Q[3]) ;
    T[0][1] =                    2*((-Q[1])*(-Q[2])-Q[0]*(-Q[3]));
    T[0][2] =                    2*((-Q[1])*(-Q[3])+Q[0]*(-Q[2]));

    T[1][0] =                    2*((-Q[1])*(-Q[2])+Q[0]*(-Q[3]));
    T[1][1] =   Q[0]*Q[0]-(-Q[1])*(-Q[1])+(-Q[2])*(-Q[2])-(-Q[3])*(-Q[3]) ;
    T[1][2] =                    2*((-Q[2])*(-Q[3])-Q[0]*(-Q[1]));

    T[2][0] =                    2*((-Q[1])*(-Q[3])-Q[0]*(-Q[2]));
    T[2][1] =                    2*((-Q[2])*(-Q[3])+Q[0]*(-Q[1]));
    T[2][2] =   Q[0]*Q[0]-(-Q[1])*(-Q[1])-(-Q[2])*(-Q[2])+(-Q[3])*(-Q[3]) ;

    Pitch  = atan( T[2][1]/T[2][2]);  
    Yaw    = asin(-T[2][0]);
    Roll   = atan( T[1][0]/T[0][0]);
 
    if(T[2][2]<0)
    {
        if(Yaw  < 0)
        {
           Yaw = Yaw +3.1416;
        }
        else
        {
           Yaw = Yaw -3.1416;
        }
    }

    if(T[0][0]<0)
    {
        if(T[1][0]>0)
        {
            Roll = Roll + 3.1416;
        }
        else
        {
            Roll = Roll - 3.1416;
        }
    }
    
    e[0]=Pitch;  //x
    e[1]=Yaw;     //y
    e[2]=Roll;
}

/*********************** oula to mouse ****************************************/
int8_t filter_x(float inp_val)
{

     float xx=(inp_val-last_val);
     last_val=inp_val;
     return (int8_t)((int)(xx));
}

int8_t filter_y(float inp_val)
{
     float yy=(inp_val-last_val2);
     last_val2=inp_val;
     return (int8_t)((int)(yy));
}


void printEvent(const sensorhub_Event_t * event)
{
    float scaleQ14 = 1.0f / (1 << 14);
    float scaleQ8 = 1.0f / (1 << 8);
    float scaleQ9 = 1.0f / (1 << 9);
    float scaleQ5 = 1.0f / (1 << 5);
    float r, i, j, k;
    int16_t x,y,z;
    int8_t xh,xl,yh,yl,zh,zl;
    int8_t msg[40],dat[30];
    msg[0]=0xa5;
    int tmp;
        switch (event->sensor) {
    case SENSORHUB_RAW_ACCELEROMETER:
        printf("Raw acc: %d %d %d\n",
               event->un.rawAccelerometer.x,
               event->un.rawAccelerometer.y, 
               event->un.rawAccelerometer.z);
        break;
        
    case SENSORHUB_ACCELEROMETER:
        x=event->un.accelerometer.x_16Q8;
        y=event->un.accelerometer.y_16Q8;
        z=event->un.accelerometer.z_16Q8;
        xh=(int8_t)(x>>8);xl=(int8_t)x; yh=(int8_t)(y>>8);yl=(int8_t)y; zh=(int8_t)(z>>8);zl=(int8_t)z;
        
   //    float_char((float)(scaleQ8*x),dat);       //x data
       // msg[1]=0xAF;msg[2]=0x04; msg[3]=dat[3];msg[4]=dat[2];msg[5]=dat[1];msg[6]=dat[0];msg[7]=0x07;
      // HAL_UART_Transmit(&huart2, msg, 6,10); 
        
     //  float_char((float)(scaleQ8*y),dat);       //y data
      //   msg[3]=dat[3];msg[4]=dat[2];msg[5]=dat[1];msg[6]=dat[0];  
        
     //  float_char((float)(scaleQ8*z),dat);       //z data
      //  msg[1]=0xA3; msg[3]=dat[3];msg[4]=dat[2];msg[5]=dat[1];msg[6]=dat[0];
        
         msg[1]=0xAF; msg[2]=0x1C;
         msg[3]=xh;msg[4]=xl;msg[5]=yh;msg[6]=yl;msg[7]=zh;msg[8]=zl;
         for(tmp=9;tmp<31;tmp++)msg[tmp]=0x22;
         msg[31]=0x1F;
        HAL_UART_Transmit(&huart2, msg, 32,10);  
      // printf("Acc: %5.3f %5.3f %5.3f\n",scaleQ8*event->un.accelerometer.x_16Q8,scaleQ8*event->un.accelerometer.y_16Q8,scaleQ8*event->un.accelerometer.z_16Q8);
        break;
     
     case SENSORHUB_GYROSCOPE_CALIBRATED:
        x=event->un.gyroscope.x_16Q9;
        y=event->un.gyroscope.y_16Q9;
        z=event->un.gyroscope.z_16Q9;
        xh=(int8_t)(x>>8);xl=(int8_t)x; yh=(int8_t)(y>>8);yl=(int8_t)y; zh=(int8_t)(z>>8);zl=(int8_t)z;

       float_char((float)(scaleQ9*x),dat);       //x data
        msg[1]=0xA1;msg[2]=0x04; msg[3]=dat[3];msg[4]=dat[2];msg[5]=dat[1];msg[6]=dat[0];msg[7]=0x07;
       HAL_UART_Transmit(&huart2, msg, 6,10); 
        
       float_char((float)(scaleQ9*y),dat);       //y data
        msg[1]=0xA2; msg[3]=dat[3];msg[4]=dat[2];msg[5]=dat[1];msg[6]=dat[0];
        HAL_UART_Transmit(&huart2, msg, 6,10);  
        
       float_char((float)(scaleQ9*z),dat);       //z data
        msg[1]=0xA3; msg[3]=dat[3];msg[4]=dat[2];msg[5]=dat[1];msg[6]=dat[0];
        HAL_UART_Transmit(&huart2, msg, 6,10);  
 
      // printf("Gyrop: %5.3f %5.3f %5.3f\n",scaleQ9*event->un.gyroscope.x_16Q9,scaleQ9*event->un.gyroscope.y_16Q9,scaleQ9*event->un.gyroscope.z_16Q9);
        break;    
        
     case SENSORHUB_MAGNETIC_FIELD_CALIBRATED:
        x=event->un.magneticField.x_16Q4;
        y=event->un.magneticField.y_16Q4;
        z=event->un.magneticField.z_16Q4;
        xh=(int8_t)(x>>8);xl=(int8_t)x; yh=(int8_t)(y>>8);yl=(int8_t)y; zh=(int8_t)(z>>8);zl=(int8_t)z;

       float_char((float)(scaleQ5*x),dat);       //x data
        msg[1]=0xA1;msg[2]=0x04; msg[3]=dat[3];msg[4]=dat[2];msg[5]=dat[1];msg[6]=dat[0];msg[7]=0x07;
       HAL_UART_Transmit(&huart2, msg, 6,10); 
        
       float_char((float)(scaleQ5*y),dat);       //y data
        msg[1]=0xA2; msg[3]=dat[3];msg[4]=dat[2];msg[5]=dat[1];msg[6]=dat[0];
        HAL_UART_Transmit(&huart2, msg, 6,10);  
        
       float_char((float)(scaleQ5*z),dat);       //z data
        msg[1]=0xA3; msg[3]=dat[3];msg[4]=dat[2];msg[5]=dat[1];msg[6]=dat[0];
        HAL_UART_Transmit(&huart2, msg, 6,10);  
       
        //printf("Magnt: %5.3f %5.3f %5.3f\n",scaleQ5*event->un.magneticField.x_16Q4,scaleQ5*event->un.magneticField.y_16Q4,scaleQ5*event->un.magneticField.z_16Q4);
        break;    
                   
    case SENSORHUB_ROTATION_VECTOR:
        r = scaleQ14 * event->un.rotationVector.real_16Q14;
        i = scaleQ14 * event->un.rotationVector.i_16Q14;
        j = scaleQ14 * event->un.rotationVector.j_16Q14;
        k = scaleQ14 * event->un.rotationVector.k_16Q14;
        
        float Quart[4];float Euler[3];
        Quart[0]=i;Quart[1]=j;Quart[2]=k;Quart[3]=r;

        Quat2Euler(Quart,Euler);   
       // printf("Orientation: r:%5.3f i:%5.3f j:%5.3f k:%5.3f \n", r, i, j, k);
       // printf("Pitch:%5.3f Yaw:%5.3f Roll:%5.3f\n",Euler[0]*57,Euler[1]*57,Euler[2]*57);
#if 0        
        int16_t yaw=Euler[1]*57*10;
        int16_t pitch=Euler[0]*57*10;
        int16_t roll=Euler[2]*57*10;
        int16_t temp=0xaF+2;   //总长度
	int8_t ctemp;
          
        msg[1]=0x5a; msg[2]=16; msg[3]=0xa1;
        if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;             //16bit的int型分两次发，先发高8位，后发低8位
        msg[4]=ctemp;
	temp+=ctemp;
	ctemp=yaw;
        msg[5]=ctemp;
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	msg[6]=ctemp;
	temp+=ctemp;
	ctemp=pitch;
	msg[7]=ctemp;
	temp+=ctemp;

	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	msg[8]=ctemp;
	temp+=ctemp;
	ctemp=roll;
	msg[9]=ctemp;
	temp+=ctemp;
        
        int16_t alt=0x1111;
	if(alt<0)alt=32768-alt;
	ctemp=alt>>8;
	msg[10]=ctemp;
	temp+=ctemp;
	ctemp=alt;
	msg[11]=ctemp;
	temp+=ctemp;

        int16_t tempr=0x1111;
	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	msg[12]=ctemp;
	temp+=ctemp;
	ctemp=tempr;
	msg[13]=ctemp;
	temp+=ctemp;

        int16_t press=0x1111;
	if(press<0)press=32768-press;
	ctemp=press>>8;
	msg[14]=ctemp;
	temp+=ctemp;
	ctemp=press;
	msg[15]=ctemp;
	temp+=ctemp;
        
        msg[16]=temp%256;
        msg[17]=0xaa;
        
        HAL_UART_Transmit(&huart2, msg, 32,10);  
#endif        
        //printf("r:%f i:%f j:%f k:%f\n",r,i,j,k);
        //printf("roll:%f pitch:%f yaw:%f\n",Euler[0]*57,Euler[1]*57,Euler[2]*57);  
        float_char(i,send_buf);
        float_char(j,send_buf+4);
        float_char(k,send_buf+8);
        float_char(r,send_buf+12);
        
        USBD_HID_SendReport(&hUsbDeviceFS,send_buf,16);
        break;
        
    default:
        printf("Unknown sensor: %d\n", event->sensor);
        break;
    }
}
