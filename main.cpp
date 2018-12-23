#include "mbed.h"
#include "MMA7660.h"
#include "C12832.h"
#include "TextLCD.h"
#include "LM75B.h"
#include "MSCFileSystem.h"

DigitalOut myled(LED1); //Setup for digital Pin
//Indicator LEDS
DigitalOut write_completed(LED2);
DigitalOut acc_usb_connected(LED3);
DigitalOut min_vib_indicator(LED4);

AnalogIn pot1(p19); //Setup for pot
AnalogIn pot2(p20);

//RGB LED
PwmOut r(p23); 
PwmOut g(p24);
PwmOut b(p25);

PwmOut speaker(p26); //Speaker

LM75B temp(p28,p27); //Temperature

MMA7660 MMA(p28, p27); //I2C Accelerometer

C12832 lcd(p5, p7, p6, p8, p11);//LCD

MSCFileSystem fs("fs"); //USB

//Thresholds for vibration and tempature
float vib_thresh_start = 2;
float min_vib_thresh = 0.16;
float vib_thresh;
float temp_thresh_start = 60;
float temp_thresh;

float * acc_xyz_start; //Varibles that will hold the accelerometer values
float * acc_xyz_end;

float x = 0, y = 0, z = 0; //The differences between the 2 accelerometer values. The vibration

float total_vib, total_vib_activation; //Total vibration and total vibration when over threshold

int choice = 0; //To pick what to show on LCD

Ticker first_read;   //Ticker to read accelerometer first time
Timeout second_read;  //Timeout to read accelerometer second time
Timer debounce;  //debouncer
Ticker vib_thresh_over; //Ticker for when vibration is over vibration threshold
Ticker temp_thresh_over; //Ticker for when tempature is over tempature threshold
Timeout digitalPin_over; //Timeout for activating digital pin

FILE *fp; //Use to open, write and close connection to USB
bool vib_trigger = 0, temp_trigger = 0, write_usb = 0, vibration_activation = 0, close_fp = 0; //Keep track if either have triggered

InterruptIn button(p14); //Change whats on the display
InterruptIn close_fp_button(p12); //Changes close_fp to true so connection to USB will close
InterruptIn change_min_vib_thresh(p15);//Change min_vib_thresh between 0 and 0.16

//Functions
void ISR1();
void ISR2();
void ISR3();
void set_RGB(float red , float green, float blue);
void acc_start();
void acc_end();
void vib_active();
void temp_active();
void digitalPin_active();
void write_USB();
void print();


//This is program checks vibration and tempature. If the values are greater than the thresholds, will trigger a blue/red RGB LED or/and the speaker.
//The values written to the USB are the vibration values. If vibration exceeds the threshold RGB alarm is activated.
//When the alarm activates for tempature, the total vibration from the vibration activation is used for the sound of the speaker.
//total_vib is saved into totol_vib_activation when vibration exceeds its threshold. ( 1 / ( total_vib_activation * 1000 ) )
//Not the total vibration at time of tempature activation.
//Once vibration alarm or tempature alarm on, its active till reset. This is an alarm! Does not switch itself off or reset active alarms(vibration, tempature).
//For reading in the accelerometer values, acc_start() + acc_end() ran 0.000005 and 0.000937 respectivily. Wanted to take the measurements in a short interval of 0.015.
//Used the longest time for equation => ( ( 2 * 0.009 ) + 0.015 ) = 0.033. During testing this time had no problem at all.
//For the digitalPin_active() method it ran consistant at 0.000001. I set the time to that and had no problem when testing.
//write_USB() took 0.7845 with first_read going. Detached from first_read, time was 0.767010. Stopped first_read before writing and activated after.
//NOTE: on the above: The above times was when the connection was opened and closed in the write_USB() function. As I moved them out so that all vibration readings can be saved
//the time has change to 0.000072. This remained the same not matter if first_read was going or not. I kept it the same and disable it before writing.

//I open the USB connection at the start for testing connection to USB. If no USB connection, waits for USB connection. If the connection is made,
//the connection is left open till you toggle the toggle switch down. This is so all vibration readings will be saved to the USB. 
//NOTE: only saves vibrations values if total vibration is above min_vib_thresh. This is due to there being small vibrations going on and this leaves a lot of junk values in the file on the USB.
//Added functionality to change mi_vib_thresh at runtime.

//HOW TO USE:
//Pot1 - changes the tempature threshold
//Pot2 - changes the vibration threshold
//Toggle button - down - closes the connection to USB
//toggle button - center - changes what is displayed on the LCD
//toggle button - up - changes min_vib_thresh value

//INDICATORS:
//RGB - blue - vibration has exceeded vibration threshold
//RGB - red - tempature has exceeded tempature threshold and speak will sound off
//LED1 - LED toggle as per spec. Happens after tempature exceeds its threshold
//LED2 - indicates the connection to USB has been closed
//LED3 - indicates connection to the accelerometer is working and USB has connected
//LED4 - indicates what min_vib_thresh is - OFF is 0, ON is 0.16


int main() { 
    
    set_RGB(1, 1, 1);
    
    acc_xyz_start = (float*)calloc(3, sizeof(float));
    acc_xyz_end = (float*)calloc(3, sizeof(float));
    
    lcd.printf("Connect USB");
    
    fp = fopen("/fs/Vib.txt","w");
        
    if(fp != NULL){
        
        lcd.cls();
        lcd.locate(0, 0);
        lcd.printf("USB connected\n");
    }
    
    //Program will not move forward unless USB is connected.
    
    (MMA.testConnection()) ? lcd.printf("Accelerometer connected\n") : lcd.printf("Accelerometer not connected\n");
    acc_usb_connected = 1;//Switchinf on LED on accelerometer connected
    
    wait(1);//Sometime to see what is written on the screen
    
    min_vib_indicator = 1;
    
    button.rise(&ISR1);
    close_fp_button.rise(&ISR2);
    change_min_vib_thresh.rise(&ISR3);
    
    debounce.start();  

    first_read.attach(&acc_start, 0.033);
    
    while(1){
        
        temp_thresh = temp_thresh_start * pot1.read();
        vib_thresh = ( ( vib_thresh_start - min_vib_thresh ) * pot2.read() ) + min_vib_thresh;
        
        if(close_fp){
            
            fclose(fp);
            write_completed = 1;//Switching on LED on write completed
        }
        else if(write_usb){
            
            write_usb = 0;
            write_USB();
            first_read.attach(&acc_start, 0.028);
        }
        
        print();
        wait(0.5);
    }
}

//Change the dispaly
void ISR1(){
    
    choice++;
}

//Change bool to indicate teh close of USB connection
void ISR2(){

    close_fp = 1;
}

//Change min_vib_thresh to 0 or 0.16
void ISR3(){
    
    if(min_vib_thresh == 0){
        
        min_vib_thresh = 0.16;
        min_vib_indicator = 1;
    }
    else{
        
        min_vib_thresh = 0;
        min_vib_indicator = 0;
    }
}

//Set RGB LED
void set_RGB(float red , float green, float blue){
    
    r = red;
    g = green;
    b = blue;
}

//Take first accelerometer reading
void acc_start(){ 

    if(debounce.read_ms() > 10){
        
        MMA.readData(acc_xyz_start);   
    }
    
    debounce.reset();
    second_read.attach(&acc_end, 0.015);
}

//Take second accelerometer reading check if thresholds have been met
void acc_end(){ 
    
    if(debounce.read_ms() > 10){
        
        MMA.readData(acc_xyz_end);
    }
    
    x = acc_xyz_end[0] - acc_xyz_start[0];
    y = acc_xyz_end[1] - acc_xyz_start[1];
    z = acc_xyz_end[2] - acc_xyz_start[2];
    
    total_vib = sqrt(( x * x ) + ( y * y ) + ( z * z));

    if(total_vib >= min_vib_thresh){ // I use min_vib_thresh as there is background vibration so at 0, there is a lot of junk writes to the USB
    
        first_read.detach();
        write_usb = 1;
    }

    //Check to see if temp_active() should start
    if(vib_trigger && !temp_trigger && temp.read() > temp_thresh){
            
        temp_trigger = 1;
        vib_thresh_over.detach();
        digitalPin_over.attach(&digitalPin_active, 0.00001);
        temp_thresh_over.attach(&temp_active, 0.5);
    }
    //check to see if vib_active() should start
    else if(!vib_trigger && total_vib > vib_thresh){
        
        total_vib_activation = total_vib;
        vib_trigger = 1;
        vibration_activation = 1;
        vib_thresh_over.attach(&vib_active, 0.1);
    }
}

//What will activate if vibration threshold is met
void vib_active(){ 

    set_RGB(1, 1, 0);
}

//What wil activat if tempature threshold is met
void temp_active(){

    set_RGB(0, 1, 1);
    speaker.period( 1 / (total_vib_activation * 1000) ); // Sound is based on what total vibration was when vibration was first set off
    speaker = 0.5;
}

//Activate digital pin
void digitalPin_active(){ 

    myled = 1;
}

//Save information to USB
void write_USB(){
        
    if(fp != NULL){
        
        if(vibration_activation){
        
            vibration_activation = 0;
            fprintf(fp, "Activation Readings:\r\n");
        }
        
        fprintf(fp, "xValue: %.2f\r\n", x);
        fprintf(fp, "yValue: %.2f\r\n", y);
        fprintf(fp, "zValue: %.2f\r\n", z);
        fprintf(fp, "Total Vib: %.2f\r\n\r\n", total_vib);
    }
}

//What will be displayed on LCD
void print(){
    
    if(choice > 2)
        choice = 0;
    
    switch(choice){
        
        case 0:
            lcd.cls();
            lcd.locate(0,0);
            lcd.printf("Vib Threshold: %.2f\n", vib_thresh);
            lcd.printf("Temp Threshold: %.2f\n", temp_thresh);
            lcd.printf("Min vib: %.2f\n", min_vib_thresh);
            break;
        
        case 1:
            lcd.cls();
            lcd.locate(0,0);
            lcd.printf("Current Vib: %.2f\n", total_vib);
            lcd.printf("Current Temp: %.2f\n", temp.read());
            lcd.printf("Vib(x,y,z): %.2f, %.2f, %.2f\n", x, y, z);
            break;
            
        case 2:
            lcd.cls();
            lcd.locate(0, 0);
            lcd.printf("Current Pot1 value: %.2f\n", pot1.read());
            lcd.printf("Current Pot2 value: %.2f\n", pot2.read());
            break;
            
        default:
            lcd.cls();
            lcd.printf("Out of range");
            break;
    } 
}