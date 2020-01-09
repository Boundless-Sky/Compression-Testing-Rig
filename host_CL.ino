/* 
 *  Title: host_CL - "host closed loop control"
 *  Purpose: Arduino code (Host) to test eIPMCs 
 *  Author: Justin Ngo, Finished July/16/2019
 *  University of Utah DARC Lab
 *  
 */

// Version 5: something is weird with the way keyes 234 load cell reader is obtaining data. it does random jumps which jack (xiang he, phd) is also experiencing
// Proposed to remove extraneous reads. 

#include <HX711.h>
#include <CircularBuffer.h>

//#define PRINT_SENSOR_FQ
#define CIRCULAR_BUFF

#define CIRCULAR_BUFFER_INT_SAFE
CircularBuffer<float,100> cBuff;

HX711 scale;
float calibration_factor = 390000; // placed 200g and got 0.448lbs ~= 200g and yes I did do a linear regression. thats how i got it, its somewhere. 
#define DOUT  3
#define CLK  2
#define stepPin 5
#define dirPin 6
#define SPR 6400 //Steps Per Revolution // see datasheet for NEMA 23 which is like 1.8deg / step -> 360/1.8 = 200
// yes this is steps per revolution but for full step since im doing microstepping 
// 1/32 it would 200*32
// NEMA 17 Motor has 200 full steps per revolution, thus 
//  200*32 microstepping which is 6400 microsteps per revolution 
int speedz = 45;//50;
float revs = 0.005;//1;

#ifdef PRINT_SENSOR_FQ
  float rawr = 0;
  long times;
  long lastTimez = 0;
#endif 

// controller
volatile int mode = 0; // 0 = idle, 1 = holding mode, 2 = tracking mode 
volatile float kp = 0.003; // 0.003 lvershoots 5.82 lbs // 0.001 ~0.1 rise time
volatile float ki = 0;
volatile float kd = 0.00001; // 0.00001;

volatile float ierror_last = 0;
volatile float i_error_max = 0.001;
volatile float error_last;
volatile float derr_last;

float N = 225; // filter 
volatile float force_desired = 1; //5.5 lbs ~= 2.5kg (artibitrary but i choose 5.5 lbs 
volatile int fReached = 0;


ISR(TIMER1_COMPA_vect){  //change the 0 to 1 for timer1 and 2 for timer2
   // this 200 hz interrupt is to read and send the force data back to matlab
   float error, effort, pterm, iterm, ierror, dterm, scale_val;
   long t1;
  switch (mode){
    case 0:{ // don't send
      break;
    }
    case 1:{ // send

      // blocking

      scale_val = scale.get_units();
    

     
      if ((fReached == 1) && (force_desired != 0)){
        if ((scale_val > force_desired*1.25) || (scale_val < -0.5)) {
          scale_val = force_desired;
        } 
      }
      if ((fReached == 2) && (force_desired == 0.0)){
        if ((scale_val > 0.01) || (scale_val < -0.01)){
          scale_val = force_desired;
        }
      }
   

      cBuff.unshift(scale_val);
      
      // Serial.println(scale_val); // works but generally you don't want this in here cause it also uses interrupt to send hence the circular buffer

      if (scale_val >= force_desired){
        fReached = 1;
      } 
      if ((fReached == 1) && (scale_val <= 0.01)) {
        fReached = 2;
      }
      
      error = force_desired - scale_val;
      pterm = kp * error;
      // increase p you have faster rise time but you get more oscillations
      // have i can get steady state ereror of zero....
      
      ierror = ierror_last + error*0.005; //200hz = 0.005
      if (ierror < -i_error_max){
        ierror = -i_error_max;
      }
      else if (ierror > i_error_max){
        ierror = i_error_max;
      }
      ierror_last = ierror;
      iterm = ki * ierror;

      dterm = (kd*N*(error-error_last)) + (derr_last * (1-N*0.005));
      derr_last = dterm;
      error_last = error;

      effort = pterm + iterm + dterm;
           
      if (effort > 0){
        moveMotor(speedz, effort,true); //up
      }
        else{
        moveMotor(speedz, -effort, false); //down
      } 
      break;
    }
    default:{
      break;
    }
  }
}

void setup(){
  cli();//stop interrupts
  // timer 1 is a 16 bit timer
  // 16mhz/(64*200hz) - 1 = 1249 for compoare match register so use with timer 1
  // 2499 for 100 hz more steady state error 0.03 instead of 0.01 (200 hz)
  // 624 for 400 hz
  //set timer1 interrupt at 200Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // turn on CTC mode
  TCCR1B |= (1 << WGM12); 
  // 256 prescaler
  TCCR1B |= (1 << CS12);  // for 50 hz
  // set compare match register for 1hz increments
  OCR1A = 1249;// = (16*10^6) / (50*256) - 1 (must be <65536), 50 hz
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();//allow interrupts

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  // debug LED
  pinMode(LED_BUILTIN, OUTPUT);

  scale.begin(DOUT, CLK);
  scale.set_scale();
  scale.tare(); //Reset the scale to 0  
  scale.set_scale(calibration_factor); //Adjust to this calibration factor
    
  Serial.begin(230400); // do not use 0 and 1 on UNO since they are connected to TX/RX
}// end setup

void loop(){
  
  char charCmd;
  String serialReceived;
  float loadcell = 0;

  #ifdef PRINT_SENSOR_FQ
    times = millis();
    rawr = scale.get_units();
    Serial.println(times - lastTimez);
    lastTimez = times;
  #endif

  #ifdef CIRCULAR_BUFF
    if (!cBuff.isEmpty()){
      Serial.println(cBuff.pop());
    }
  #endif
  
  if (Serial.available() && (mode == 1) && (fReached == 1)){
    force_desired = 0; // decompress. send anything
  }
  
  if (Serial.available() && (mode == 1) && (fReached == 2)){
    fReached = 0;
    mode = 0; //turn off
  }
  
  if (Serial.available()) {
  //if (Serial.available() && (mode == 0)){
    serialReceived = Serial.readStringUntil('\n');
    charCmd = serialReceived.charAt(0);
    
    switch (charCmd){
      case 'w': // up 
      {
        moveMotor(speedz,revs,true);
      
        Serial.flush();
        break;
      }
      case 's': // down
      {
        moveMotor(speedz,revs,false);
        Serial.flush();
        break;
      }
      case 'a': // set revolutions
      {
        // wait until serial is avliable then read
        while (!Serial.available()){};
        revs = Serial.parseFloat();
        break;
      }
      case 'd': // set speed
      {
        while (!Serial.available()){};
        speedz = Serial.parseInt();
        break;
      }
      case 'e': // get current revolution and speed
      {
        Serial.println(revs);
        Serial.println(speedz);
        break;
      }
      case 'r': // read current lbs
      {
        loadcell = scale.get_units();
        Serial.println(loadcell);
        break;
      }
      case 't': // set calibartion factor or Reset
      {
        while (!Serial.available()){};
        calibration_factor = Serial.parseInt();
        break;
      }
      case 'y': // read stream of load cell data lbs
      {
        for (int i = 0; i< 51; i++){
          loadcell = scale.get_units();
          Serial.println(loadcell);
        }
        break;
      }
      case 'm': // test 1 - compress
      {
        while (loadcell < 5.3){
          moveMotor(speedz,revs,true);
          loadcell = scale.get_units();
          Serial.println(loadcell);
        }
        moveMotor(speedz,revs,false);
        break;
      }
      case 'n': //
      {
        while (loadcell > 0.005){ // instead of this i can have a counter in the
          // case m and when it is tim to unwind i can run move motor the same 
          // amount of time. 
          moveMotor(speedz,revs,false);
          loadcell = scale.get_units();
          Serial.println(loadcell);
        }
        break;
      }
      case 'f': // turn on controller
      {
        while (!Serial.available()){}; // wait for compression 
        force_desired = Serial.parseFloat();
        Serial.read();
        mode = 1;     
        break;
      }
      default:
      {
        break;
      }
    }
  }// serial.avaliable
} // main loop

/*~~~~~~~~~~~~~~~~~~~~ FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void moveMotor(int velocity, float revolution, bool motion){
  // Velocity: 50 is *fastest (needs more testing) (50? to 2000?)
  
  // motion: UP - true, DOWN - false
  if (motion == true) { // UP
    digitalWrite(dirPin, HIGH); // High - CW(up), Low -CCW(down)
    // higher frequency faster the motor spins
    // can use analogWrite(0 to 255) on pins with ~ so would have to change stepPin to i.e 3
  } else { // DOWN
    digitalWrite(dirPin, LOW);
  }
   for (int i = 0; i < SPR*revolution; i++){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(velocity); // 50 gives fastest frequency so far
    digitalWrite(stepPin, LOW);
    delayMicroseconds(velocity);
  }
} // end moveMotor
