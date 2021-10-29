/* Library */
#include "project.h"
#include <stdio.h>

int state=99; 
char string_1[20];
char string_2[20];

/* Color Sensor*/
int color_L_puck1=0;
int color_R_puck1=0;
int color_L_puck2=0;
int color_R_puck2=0;
int color_L_zone1=3; //G
int color_R_zone1=2; //B
int color_L_zone2=1; //R
int color_R_zone2=3; //G

int mode=1;
int compare_ready=0;
int mode2=1;
int compare_ready2=0;
uint count=0;
int allmatch=0;

/* Motor Speed*/ 
int32 speed=35000;
int32 speed3=27500;
int32 speedg=6000;
int32 speed2=22000;
int32 speed5=10000;
/* Ultrasonic sensor*/
float distance_measured=0; //ultrasonic 
uint16 Ultracount=0; 
float distance_measured2=0; //ultrasonic 
uint16 Ultracount2=0; 
int next=0;
int next_round=0;

/* Distance */

int difference=1500; 
uint16 distance_puck1=6000;
uint16 distance_puck2=1450;
uint16 distance_ready2=3600;
uint16 distance_ready2b=75;//170
uint16 distance_back=60000;


/* Servo Motor */
uint aL1=980; //90 
uint bL1=930;
int cL=1;

uint aR1=980; //90
uint bR1=1030;
int cR=1;

uint aL2a=930; //45 preparation //if match and flick
uint aL2b=980; //if not match and flick
uint bL2=950; 
int cL2=1;

uint aR2a=1030; //45 preparation //if match and flick
uint aR2b=980; //if not match and flick
uint bR2=1025;
int cR2=1;

uint bR3=1045;
uint bL3=905;

int zone5=0;
int zone6=0;
int zone7=0;
int zone8=0;

//*****************************************************// 
// STOP
//*****************************************************//
void stop2()
{
    Motor_L_IN_1_Write(0);
    Motor_L_IN_2_Write(0);
    Motor_R_IN_3_Write(0);
    Motor_R_IN_4_Write(0);
    PWM_Motor_L_WriteCompare(0);
    //CyDelay(100);
    PWM_Motor_R_WriteCompare(0);
    
    
    QuadDec_L_SetCounter(0);
    //CyDelay(100);
    
    QuadDec_R_SetCounter(0);
}


void stop()
{
    Motor_L_IN_1_Write(0);
    Motor_L_IN_2_Write(0);
    Motor_R_IN_3_Write(0);
    Motor_R_IN_4_Write(0);
    PWM_Motor_L_WriteCompare(0);
    //CyDelay(95);
    PWM_Motor_R_WriteCompare(0);
    
    
    QuadDec_L_SetCounter(0);
    //CyDelay(95);
    
    QuadDec_R_SetCounter(0);
}

//*****************************************************// 
// MOTION
//*****************************************************//

void forward(int speed, int difference)
{
    Motor_L_IN_1_Write(1);
    Motor_L_IN_2_Write(0);
    Motor_R_IN_3_Write(1);
    Motor_R_IN_4_Write(0);
    if (QuadDec_L_GetCounter() - QuadDec_R_GetCounter()>= difference) // R as master L as slave
    {
        PWM_Motor_L_WriteCompare(speed-348);  //350
        PWM_Motor_R_WriteCompare(speed);
    }
    else if (QuadDec_L_GetCounter() - QuadDec_R_GetCounter()<= -difference)
    {
        PWM_Motor_L_WriteCompare(speed+348); //320
        PWM_Motor_R_WriteCompare(speed);
    }
    else {
        PWM_Motor_L_WriteCompare(speed);
        PWM_Motor_R_WriteCompare(speed);
    }
    
}

void backward(int speed, int difference)
{
    Motor_L_IN_1_Write(0);
    Motor_L_IN_2_Write(1);
    Motor_R_IN_3_Write(0);
    Motor_R_IN_4_Write(1);
    if (QuadDec_L_GetCounter()- QuadDec_R_GetCounter()<= -difference) // R as master L as slave
    {
        PWM_Motor_L_WriteCompare(speed-400); 
        PWM_Motor_R_WriteCompare(speed);
    }
    else if (QuadDec_L_GetCounter() - QuadDec_R_GetCounter()>= difference)
    {
        PWM_Motor_L_WriteCompare(speed+400);
        PWM_Motor_R_WriteCompare(speed);
    }
    else {
        PWM_Motor_L_WriteCompare(speed);
        PWM_Motor_R_WriteCompare(speed);
    }
    
}

//*****************************************************// 
// PREDEFINED DISTANCE
//*****************************************************// 
void forward_Distance(int speed, int difference, int distance) //Right wheel as Master while Left wheel as Slave
{
    Motor_L_IN_1_Write(1);
    Motor_L_IN_2_Write(0);
    Motor_R_IN_3_Write(1);
    Motor_R_IN_4_Write(0);
    while (QuadDec_L_GetCounter() <= distance) {
    // Keep regulating the speed until there is no need for forward motion 
        if (QuadDec_L_GetCounter() - QuadDec_R_GetCounter() >= difference) {  //threshold difference to be adjusted //if left outrun right left decrease
           PWM_Motor_L_WriteCompare(speed-349); // kp*difference -> preset difference eg: 3000
            PWM_Motor_R_WriteCompare(speed);
        }
        else if (QuadDec_L_GetCounter() - QuadDec_R_GetCounter() <= -difference) {  //if right outrun left left increase
            PWM_Motor_L_WriteCompare(speed+349); //320
            PWM_Motor_R_WriteCompare(speed);
        }
        else {
    // Make the motor run forward 
            PWM_Motor_L_WriteCompare(speed); 
            PWM_Motor_R_WriteCompare(speed);
        } 
    }
    // Stop
    stop();
}

void backward_Distance(int speed, int difference, int distance)
{
    Motor_L_IN_1_Write(0);
    Motor_L_IN_2_Write(1);
    Motor_R_IN_3_Write(0);
    Motor_R_IN_4_Write(1);
    while (QuadDec_L_GetCounter() >= -distance) { 
    
        if (QuadDec_L_GetCounter() - QuadDec_R_GetCounter() <= -difference) { //left more negative =left faster -> decrease Left speed
            PWM_Motor_L_WriteCompare(speed-335); 
            PWM_Motor_R_WriteCompare(speed);
        }
        else if (QuadDec_L_GetCounter() - QuadDec_R_GetCounter() >= difference) { //right more negative = left slow -> increase left speed
            PWM_Motor_L_WriteCompare(speed+335); 
            PWM_Motor_R_WriteCompare(speed);
        }
        else {
    
            PWM_Motor_L_WriteCompare(speed); 
            PWM_Motor_R_WriteCompare(speed);
        } 
    }
    
    stop();
}

//*****************************************************// 
// TURNING
//*****************************************************//


void turn_L_forward(int speed)
{
    stop();
    Motor_L_IN_1_Write(0);
    Motor_L_IN_2_Write(1);
    Motor_R_IN_3_Write(0);
    Motor_R_IN_4_Write(1);
    
    while (QuadDec_L_GetCounter() >= -1 ) { //-36500 360degree
        PWM_Motor_L_WriteCompare(speed); 
    }
    //PWM_Motor_L_WriteCompare(speed); 
    //stop(); 
}
void turn_L_backward(int speed) 
{
    Motor_L_IN_1_Write(1);
    Motor_L_IN_2_Write(0);
    Motor_R_IN_3_Write(1);
    Motor_R_IN_4_Write(0);
    
        while (QuadDec_L_GetCounter() <= 100) //36000 360degree
        {
            PWM_Motor_L_WriteCompare(speed);
            } 
    //PWM_Motor_L_WriteCompare(speed);
    //stop();
}

void turn_R_forward(int speed)
{
    Motor_L_IN_1_Write(0);
    Motor_L_IN_2_Write(1);
    Motor_R_IN_3_Write(0);
    Motor_R_IN_4_Write(1);
    while (QuadDec_R_GetCounter() >= -7000 ) {
            PWM_Motor_R_WriteCompare(speed);
            } 
    //PWM_Motor_R_WriteCompare(speed);
       // stop();
}

void turn_R_backward(int speed) 
{
    Motor_L_IN_1_Write(1);
    Motor_L_IN_2_Write(0);
    Motor_R_IN_3_Write(1);
    Motor_R_IN_4_Write(0);

        while (QuadDec_R_GetCounter() <= 125 ) {
            PWM_Motor_R_WriteCompare(speed);
            } 
    //PWM_Motor_R_WriteCompare(speed);
        //stop();
}
//*****************************************************//
// COLOR SENSOR
//*****************************************************//
int colorSense(void)
{
    int R=0;
    int B=0;
    int G=0;
    int count=0;
    int sign=0;
        
    //CyDelay(10);
    
    if (mode==1)
    {
        
        S2_Write(0);
        S3_Write(0);
        CyDelay(20);
        Control_Reg_1_Write(1);
        CyDelay(1);
        Control_Reg_1_Write(0);
        while (compare_ready == 0)
        {
            
        }
        count=Counter_1_ReadCapture();
        R=count;
        sprintf(string_1,"red: %d\n", count);
        //UART_1_PutString(string_1);
        mode=2;
        compare_ready=0;
        
    }
    
     if (mode==2)
    {
        S2_Write(0);
        S3_Write(1);
        CyDelay(20);
        Control_Reg_1_Write(1);
        CyDelay(1);
        Control_Reg_1_Write(0);
        while (compare_ready == 0)
        {
            
        }
        count=Counter_1_ReadCapture(); 
        B=count;
        sprintf(string_1,"blue: %d\n", count);
        //UART_1_PutString(string_1);
        mode=3;
        compare_ready=0;
        
        
    }
    
    if (mode==3)
    {
        S2_Write(1);
        S3_Write(1);
        CyDelay(20);
        Control_Reg_1_Write(1);
        CyDelay(1);
        Control_Reg_1_Write(0);
        while (compare_ready == 0)
        {
            
        }
        count=Counter_1_ReadCapture(); 
        G=count;
        sprintf(string_1,"green: %d\n", count);
        //UART_1_PutString(string_1);
        mode=1;
        compare_ready=0;
    }
    
    
    if((R>G)&& (R>B)){
        sign=1;  //Red
    }
    else if((G>R) && (G>B)){
        sign=3;         //Green
    } 
    else if ((B>R) && (B>G)){
        sign=2;        //Blue 
    }
    return sign;
}

int colorSense2(void)
{
    int R=0;
    int B=0;
    int G=0;
    int count=0;
    int sign=0;
        
    //CyDelay(10);
    
    if (mode2==1)
    {
        
        S2_1_Write(0);
        S3_1_Write(0);
        CyDelay(20);
        Control_Reg_2_Write(1);
        CyDelay(1);
        Control_Reg_2_Write(0);
        while (compare_ready2 == 0)
        {
            
        }
        count=Counter_2_ReadCapture();
        R=count;
        sprintf(string_1,"red: %d\n", count);
        //UART_1_PutString(string_1);
        mode2=2;
        compare_ready2=0;
        
    }
    
     if (mode2==2)
    {
        S2_1_Write(0);
        S3_1_Write(1);
        CyDelay(20);
        Control_Reg_2_Write(1);
        CyDelay(1);
        Control_Reg_2_Write(0);
        while (compare_ready2 == 0)
        {
            
        }
        count=Counter_2_ReadCapture(); 
        B=count;
        sprintf(string_1,"blue: %d\n", count);
        //UART_1_PutString(string_1);
        mode2=3;
        compare_ready2=0;
        
        
    }
    
    if (mode2==3)
    {
        S2_1_Write(1);
        S3_1_Write(1);
        CyDelay(20);
        Control_Reg_2_Write(1);
        CyDelay(1);
        Control_Reg_2_Write(0);
        while (compare_ready2== 0)
        {
            
        }
        count=Counter_2_ReadCapture(); 
        G=count;
        sprintf(string_1,"green: %d\n", count);
        //UART_1_PutString(string_1);
        mode2=1;
        compare_ready2=0;
    }
    
    
    if((R>G)&& (R>B)){
        sign=1;  //Red
    }
    else if((G>R) && (G>B)){
        sign=3;         //Green
    } 
    else if ((B>R) && (B>G)){
        sign=2;        //Blue 
    }
    return sign;
}
CY_ISR(Color_L_ISR_Handler)
{
    PWM_Color_L_ReadStatusRegister();
    compare_ready=1;
}

CY_ISR(Color_R_ISR_Handler)
{
    PWM_Color_R_ReadStatusRegister();
    compare_ready2=1;
}

//********LED***************//
void color_show1(int color) 
{
    int d=0;
    if(color==1) {
        d=1;//R 
    }
    else if(color==3) {
        d=3; //G
        }
    else if(color==2) {
        d=2; //B
        }
     
    for(int i=0;i<d;i++)
    {
    LED_OUT_Write(1);
    /* Place your application code here. */
    CyDelay(500);
    LED_OUT_Write(0);
    CyDelay(500);
    }
    d=0;
}


void IRsense(void)
{
    for (;;){
        if(state==0){
        if(IR_Read()==0){
            stop2();
            break;
        }
        else if(IR_Read()==1){
            forward(speed,difference);
        }
        }
        
        if(state==3){
        if(IR_Read()==0){
            stop();
            break;
        }
        else if(IR_Read()==1){
            forward(speed,difference);
        }
        }
        
    }
}

//*****************************************************// 
// Main
//*****************************************************//
int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    for(;;)
    {
        
        if(Push_Read()==0)
        {
            state=0;
            QuadDec_L_Start();
            QuadDec_R_Start();
            PWM_Motor_L_Start();
            
            
            
            PWM_Servo_LF_Start();
            PWM_Servo_RF_Start();
            PWM_Servo_LB_Start();
            PWM_Servo_RightBack_Start();
            PWM_Motor_R_Start();
            CyDelay(2000);
            
        }      
        else {
            stop();
            
        }
         switch(state)
        {
            case 0: //IR sense puck and stop else move forward
            {
                //PWM_Motor_L_WriteCompare(20000);
                //turn_L_backward(speed);
                IRsense();
                state=1;
                break;
            }
            
            case 1: //sense color
            {            
                color_L_puck1=0;
                color_R_puck1=0;
                
                PWM_Color_L_Start();
                Counter_1_Start();   
                LED_Write(1);              
                S0_Write(1);  
                S1_Write(1);                
                CyDelay(20);                 
                isr_2_StartEx(Color_L_ISR_Handler);                
                compare_ready=0; 
                mode=1;
                color_L_puck1=colorSense();
                CyDelay(20);
                
                PWM_Color_R_Start();
                Counter_2_Start();
                LED_1_Write(1);              
                S0_1_Write(1);  
                S1_1_Write(1);
                CyDelay(20);
                isr_3_StartEx(Color_R_ISR_Handler);
                compare_ready2=0; 
                mode2=1;
                color_R_puck1=colorSense2();
                //color_show1(color_L_puck1); //to check
                LED_1_Write(0);
                LED_Write(0);
                state=2;               
                //CyDelay(1000); //@@
                break;
               
            }
            case 2: //i)compare the color of puck zone  ii)flick 
            {
            forward_Distance(speed,difference,distance_ready2);
            turn_L_forward(speedg);
            //Preparation
            while (PWM_Servo_LF_ReadCompare()>=bL1) //930
            {
                    PWM_Servo_LF_WriteCompare(aL1-cL); //980-x
                    cL+=3; //Improve
                    CyDelay(20);
                    
                }
            
            
            
            while (PWM_Servo_RF_ReadCompare()<=bR1){     //1030
                    PWM_Servo_RF_WriteCompare(aR1+cR); //980 +x
                    cR+=3;
                    CyDelay(20);
                    
                }
            
            
                
            
              
            stop();    
            CyDelay(500); //@@
               state=3;
               break;
            }
            
            case 3: //i) predefined distance(forward)ready  ii)ready servo
            {
               IRsense();
                
                color_L_puck2=0;
                color_R_puck2=0;
                
                PWM_Color_L_Start();
                Counter_1_Start();   
                LED_Write(1);              
                S0_Write(1);  
                S1_Write(1);                
                CyDelay(20);                 
                isr_2_StartEx(Color_L_ISR_Handler);                
                compare_ready=0; 
                mode=1;
                color_L_puck2=colorSense();
                CyDelay(20);
                
                PWM_Color_R_Start();
                Counter_2_Start();
                LED_1_Write(1);              
                S0_1_Write(1);  
                S1_1_Write(1);
                CyDelay(20);
                isr_3_StartEx(Color_R_ISR_Handler);
                compare_ready2=0; 
                mode2=1;
                color_R_puck2=colorSense2();
                //color_show1(color_R_puck); //check
                LED_1_Write(0);
                LED_Write(0);
                state=4;
                
                cL=1;
                cR=1;
            
            CyDelay(500);
                backward_Distance(speed2, difference, distance_ready2b);
                stop();
                CyDelay(200);
                state=4;
                break;
            }
            
            case 4: //i) predefined distance(forward)to puck ii) sense color
            {
                
                
                
                if(color_L_puck1==color_L_zone1){
                
                while (PWM_Servo_LB_ReadCompare()>=bL1){  //930
                    PWM_Servo_LB_WriteCompare(aL1-cL); //980-3
                    cL+=5;
                    CyDelay(20);
                    
                }
                }
                CyDelay(950);
                if(color_R_puck1==color_R_zone1){
                
                while (PWM_Servo_RightBack_ReadCompare()<=bR1){ ////1030
                    PWM_Servo_RightBack_WriteCompare(aR1+cR); //980+3
                    cR+=6;
                    CyDelay(20);
                    
                }
                }
                CyDelay(950);
                cL=1;
                cR=1;
                
                if(color_L_puck2==color_L_zone2){
                
                while (PWM_Servo_LF_ReadCompare()<=aL1){  //980
                    PWM_Servo_LF_WriteCompare(bL1+cL); //955+3
                    cL+=3;
                    CyDelay(20);
                    
                }
                }
                CyDelay(950);
                if(color_R_puck2==color_R_zone2){
                
                while (PWM_Servo_RF_ReadCompare()>=aR1){ ////980
                    PWM_Servo_RF_WriteCompare(bR1-cR); //1005-3
                    cR+=5;
                    CyDelay(20);
                    
                }
                }
                stop();
                state=5;
                CyDelay(600); //@@
                break;  
                
            }

            case 5: // ii) flick the puck   
            {
                cL=1;
                cR=1;
                
                if(color_L_puck1==color_L_zone1){
                
                while (PWM_Servo_LB_ReadCompare()<=aL1){  //980
                    PWM_Servo_LB_WriteCompare(bL1+cL); //930+3
                    cL+=5;
                    CyDelay(20);
                    
                }
                }
                if(color_R_puck1==color_R_zone1){
                
                while (PWM_Servo_RightBack_ReadCompare()>=aR1){ ////980
                    PWM_Servo_RightBack_WriteCompare(bR1-cR); //1030-3
                    cR+=5;
                    CyDelay(20);
                    
                }
                }
                cL=1;
                cR=1;
                
                if(color_L_puck2==color_L_zone2){
                
                while (PWM_Servo_LF_ReadCompare()>=bL1){  //930   //<   //CONFRIMED IS >
                    PWM_Servo_LF_WriteCompare(aL1-cL); //980-3
                    cL+=5;
                    CyDelay(20);
                    
                }
                }
                if(color_R_puck2==color_R_zone2){
                
                while (PWM_Servo_RF_ReadCompare()<=bR1){ ////1030
                    PWM_Servo_RF_WriteCompare(aR1+cR); //980+3
                    cR+=5;
                    CyDelay(20);
                    
                }
                }
                stop();
                state=6;
                CyDelay(600);
                break;
            }
            
            case 6: //predefined distance backward (backward) servo back position
            {
                zone5=color_R_puck1&&color_R_zone1;
                zone6=color_L_puck1&&color_L_zone1;
                zone7=color_R_puck2&&color_R_zone2;
                zone8=color_L_puck2&&color_L_zone2;
                allmatch=color_L_puck1==color_L_zone1 && color_L_puck2==color_L_zone2  && color_R_puck1==color_R_zone1  &&color_R_puck2==color_R_zone2;
                if(allmatch ||((color_R_puck2==color_R_zone2)&&(color_L_puck2==color_L_zone2))){ //all match or 3 valid (zone 7 and zone 8 match)
                    state=9;
                CyDelay(2000);}
                else if((color_R_puck1==color_R_zone1) &&(color_L_puck1==color_L_zone1)){ //3valid pucks 
                    state=8;
                CyDelay(1200);}

                else {
                    state=7;
                CyDelay(1200);} //**adjust the delay after 1st cycle 
                /*else if ((color_R_puck1==color_R_zone1)||(color_L_puck1==color_L_zone1)||(color_R_puck2==color_R_zone2)||(color_L_puck2==color_L_zone2)){ //1 or 2 valid pucks 
                    state=7;
                CyDelay(2000);}*/
                /*else{
                    state=99;}*/
                
                
                
                break;
            }
            case 7:{  
                //1 //put confident at top first , adjust delay (after each put and back )
                if(color_L_puck1==color_L_zone1){
                for(;;){
                    cL=1;
                    while (PWM_Servo_LB_ReadCompare()>=bL1){  //930
                    PWM_Servo_LB_WriteCompare(aL1-cL); //980-3
                    cL+=5;
                    CyDelay(20);
                    
                }
                    
                
                cL=1;
                
                while (PWM_Servo_LB_ReadCompare()<=aL1){  //980
                    PWM_Servo_LB_WriteCompare(bL1+cL); //930+3
                    cL+=4;
                    CyDelay(20);
                    
                }    
                
                CyDelay(800);
                }
             
                
                }
                if(color_R_puck1==color_R_zone1){
                for(;;){
                    cR=1;
                 while (PWM_Servo_RightBack_ReadCompare()<=bR1){ ////1030
                    PWM_Servo_RightBack_WriteCompare(aR1+cR); //980+3
                    cR+=7;
                    CyDelay(20);
                    
                }   
                
                cR=1;
                    while (PWM_Servo_RightBack_ReadCompare()>=aR1){ ////980
                    PWM_Servo_RightBack_WriteCompare(bR1-cR); //1030-3
                    cR+=6;
                    CyDelay(20);
                    
                }
                
                
                CyDelay(800);
                }
                }
                
                cL=1;
                cR=1;
                
                if(color_L_puck2==color_L_zone2){
                for(;;){
                    cL=1;
                while (PWM_Servo_LF_ReadCompare()<=aL1){  //980
                    PWM_Servo_LF_WriteCompare(bL1+cL); //930+3
                    cL+=3;
                    CyDelay(20);
                    
                }
                cL=1;
                
                while (PWM_Servo_LF_ReadCompare()>=bL1){  //930   //<   //CONFRIMED IS >
                    PWM_Servo_LF_WriteCompare(aL1-cL); //980-3
                    cL+=5;
                    CyDelay(20);
                    
                }
                CyDelay(800);
                }
                }
                
                if(color_R_puck2==color_R_zone2){
                for(;;){
                    cR=1;
                while (PWM_Servo_RF_ReadCompare()>=aR1){ ////980
                    PWM_Servo_RF_WriteCompare(bR1-cR); //1005-3
                    cR+=6;
                    CyDelay(20);
                    
                }
                cR=1;
                
                while (PWM_Servo_RF_ReadCompare()<=bR1){ ////1030
                    PWM_Servo_RF_WriteCompare(aR1+cR); //980+3
                    cR+=5;
                    CyDelay(20);
                    
                }
                CyDelay(800);
                }
                }
                
            }
            case 8:{ //3  valid  (5,6)
                
                for(;;)
                {
                    cL=1;
                cR=1;
                while (PWM_Servo_LB_ReadCompare()>=bL3){  //930
                    PWM_Servo_LB_WriteCompare(aL1-cL); //980-3
                    cL+=4;
                    CyDelay(20);
                    
                }
                cL=1;
                while (PWM_Servo_LB_ReadCompare()<=aL1){  //980
                    PWM_Servo_LB_WriteCompare(bL1+cL); //930+3
                    cL+=5;
                    CyDelay(20);
                    
                }
                
                
                
                CyDelay(1000); //each puck duration wait
                
                while (PWM_Servo_RightBack_ReadCompare()<=bR3){ ////1030
                    PWM_Servo_RightBack_WriteCompare(aR1+cR); //980+3
                    cR+=4;
                    CyDelay(20);
                    
                }
                
                cL=1;
                cR=1;
                
                while (PWM_Servo_RightBack_ReadCompare()>=aR1){ ////980
                    PWM_Servo_RightBack_WriteCompare(bR1-cR); //1030-3
                    cR+=7;
                    CyDelay(20);
                    
                }
                
                
                
                
                
                
                CyDelay(1300);
                }
                
                
            }
            case 9: { //4 valid
                
                for (;;) {
                    cL=1;
                    cR=1;
                    while (PWM_Servo_LF_ReadCompare()<=aL1){  //980
                    PWM_Servo_LF_WriteCompare(bL1+cL); //955+3
                    cL+=3;
                    CyDelay(20);
                    
                    
                }
                cL=1;
                    while (PWM_Servo_LF_ReadCompare()>=bL1){  //930   //<   //CONFRIMED IS >
                    PWM_Servo_LF_WriteCompare(aL1-cL); //980-3
                    cL+=5;
                    CyDelay(20);
                    
                }
                CyDelay(1000);
                
                cR=1;
                
                while (PWM_Servo_RF_ReadCompare()>=aR1){ ////980
                    PWM_Servo_RF_WriteCompare(bR1-cR); //1005-3
                    cR+=5;
                    CyDelay(20);
                    
                }
                
                
                cL=1;
                cR=1;
                
                
                
                
                while (PWM_Servo_RF_ReadCompare()<=bR1){ ////1030
                    PWM_Servo_RF_WriteCompare(aR1+cR); //980+3
                    cR+=5;
                    CyDelay(20);
                    
                }
                CyDelay(1300);
                }
                
            }
                //state=8;
                //break;
        }  
    }
}

/* [] END OF FILE */
