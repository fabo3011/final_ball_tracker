#include <Servo.h>

//3 sevos used
Servo cam_up;
Servo cam_side;

//Stored possitions
int cam_up_pos=47; //47 centered
int cam_side_pos=0;
int past_pos= 1;

int up_min= 10;
int up_max= 90;
int side_min= 0;
int side_max= 180;

char msg=0x00;
int dir= 0;
  
int follow_vel=80;
int sweep_vel=50;

 void setup() 
{
   Serial.begin(115200);
   //Pin 5 for Servo CamUp
   //Pin 6 for Servo CamSide
   cam_side.attach(5);
   cam_up.attach(6);
}
 
void loop() {

  
  //If nothing found, search
  if ( (msg&0x0F)==0x00){
    
    if (dir == 0 && cam_side_pos == side_max) {
      dir=1;}
    else if (dir == 1 && cam_side_pos == side_min) {
      dir=0;}
      
    cam_side_pos += (dir==0 && cam_side_pos<side_max);
    cam_side_pos -= (dir==1 && cam_side_pos>side_min);
    
    cam_side.write(cam_side_pos); 
    cam_up_pos=47;
     delay(sweep_vel/2); 
    cam_up.write(cam_up_pos); 
    delay(sweep_vel/2); 
  }
  
  //Follow the ball if is found
  else {
  
    cam_side_pos += (bitRead(msg, 3) && bitRead(msg, 1) && cam_side_pos<180);
    cam_side_pos -= (bitRead(msg, 3) && !bitRead(msg, 1) && cam_side_pos>0);
    
    cam_up_pos += (bitRead(msg, 2) && !bitRead(msg, 0) && cam_up_pos<up_max);
    cam_up_pos -= (bitRead(msg, 2) && bitRead(msg, 0) && cam_up_pos>up_min);
  
    cam_side.write(cam_side_pos); 
    delay(follow_vel); 
    cam_up.write(cam_up_pos); 
    delay(follow_vel*2); 
    
  }
  
}

void serialEvent () {
    msg=Serial.read();
    
    //Sweeping with threshes
    if ((msg & 0x0F) == 0x00) {
        //Adjust search range with received thresh
        if ( msg == 0) { //0 - 180
          side_min= 0;
          side_max= 180;
          sweep_vel=50;
        } 
        else if ( msg == 16 || msg == 80) { //135
          side_min= cam_side_pos-35;
          side_max= cam_side_pos+35;
          sweep_vel=75;
        } 
        else if ( msg == 32 || msg == 96 ) { //90
          side_min= cam_side_pos-20;
          side_max= cam_side_pos+20;
          sweep_vel=90;
        }
        else if ( msg == 48 || msg == 112 ) { //45
          side_min= cam_side_pos-10;
          side_max= cam_side_pos+10;
          sweep_vel=120;
        }
        
        //Adjust range between 0 and 180
        if (side_min < 0) {
          side_min-= side_min;
          side_max-= side_min;
        }
        else if (side_max>180) {
          side_min-= side_max-180;
          side_max-= side_max-180;
        }
        
        //If ball si found change direction
        if (dir != bitRead(msg, 6)) {
          dir = bitRead(msg, 6);
        }
    } 
    //Check if instruction si for sweep or follow
    else if  ( bitRead(msg, 6)) {
          //Serial.println(cam_side_pos);
            Serial.write(cam_side_pos);
    }
}

