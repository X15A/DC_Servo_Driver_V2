#include <Arduino.h>

// Quadratre Input Servo Motor (QISMO) Driver V1.3
//Written by Howard Bartlett on 4/23/2017
//Last updated by Howard Bartlett on 8/28/2024 (added some comments, removed profanity)


//Common Clause licensing terms
  /*Distributed by Causality Manufacturing under the "Common Clause" licensing definition (https://commonsclause.com/), meaning that all parties are welcome to use, modify, and share 
  changes to this source code; however this code cannot be sold without "value added". "Value added" is defined in this context as significant additional functionality via software changes, 
  or as hardware that comes pre-flashed/designed for this source code. The intention behind the use of this licensing is to allow this code to be easily accessable to hobbyists 
  while preventing possible exploitation of individuals who do not understand that it is publicly available at no cost.
  Any derivative works must be licensed under the same "Common Clause" terms, with attributions given to the original source code.*/


//Description
  /* DC Servo motor driver for stepper motor emulation. Uses any PWM or logic input motor driver such as brushed ESC's, L298, or DRV8871. Logic input drivers like the DRV8871 will have better 
  Currently configured to run on atmega328 or arduino nano. Pinout can be changed for different microcontrollers, just ensure that new pins are capable of performing required functions
  Code is not super well optimized, but runs smoothly with encoder input rates up to 200,000 pulses per second. Feel free to make improvements and share. 
  */



// Function Declarations (For Platform IO to play nice)
  void enableCheck();
  bool AlarmCheck();
  void Encoder();
  void Step();
  double PIDF();
  void motor2Pin(double power, int chA, int chB);
  void motor1Pin(double power, int pOut);
  void serialDebug();
  

// Pin map
  #define EnA 2 // Encoder pin A, must be capable of hardware interrupts (pin 2 or 3 on arduino nano)
  #define EnB 6 // Encoder pin B, does not require hardware interrupts (encoder is read in half-wave intervals)

  #define Stp 3 // Step input pin, must be capable of hardware interrupts (pin 2 or 3 on arduino nano)
  #define Dir 4 // Direction input pin, can be any digital input pin (prefer input pullup, if not, change input type in line # to INPUT)
  #define En  5 // Enable input pin, can be any digital input pin (prefer input pullup)

  #define Alarm 8 // Alarm output pin to signal error. Can be used to tell the printer that there is something wrong to stop printing.

  // Directional input driver  
  #define OutA 9 // Define unidirectional output A. Must be capable of generating PWM signals (arduino nano pins 3, 5, 6, 9, 10, 11)
  #define OutB 10 // Define unidirectional output B. Must be capable of generating PWM signals (arduino nano pins 3, 5, 6, 9, 10, 11)
  // Single wire driver
  #define outPin 11 // Define bidirectional PWM output pin; Must be capable of generating PWM signals (arduino nano pins 3, 5, 6, 9, 10, 11)

// Input Settings
  const double EnRes = 1188.0; // Encoder resolution per rotation. Not essential to have correct unless printer firmware doesn't allow you to change Esteps
  const double steprate = 1188.0; //Step resolution per rotation. Not essential to have correct unless printer firmware doesn't allow you to change Esteps. Try to keep as factor of EnRes if possible
  #define Pullup true // Use PULLUP resistors on inputs
  
  #define InvertEncoder true // inverts direction of encoder  

// Output Settings
  #define outMode 0 // Set output type to 2 channel (0) or 1 channel(1)     (2 channel for drivers like the DRV8871, 1 channel for standard pwm ESC's)
  #define EnableSafe false // Prevents any movement, encoder tracking, or target tracking when disable. HIGHLY reccomended for safety, especially with large motors.
  #define AlarmMode 0 // Sets conditions required to trigger alarm. 0 = no alarm, 1 = error threshold. 
  #define AlarmBehavior 0 // Sets behavior of servo after alarm is triggered. 0 = no changes, 1 = Estop.

  #define invertMotor true // Inverts motor output

  const int AlarmLim = 1024; // Error limit before alarm trigger. Lower values produce faster and more precise responses, but may be more likely to trigger from non catastrophic oscillations. 
// Variables




  const double EpS = EnRes/steprate; // conversion ratio between encoder pulse and step per rotation. Calculated once to limit memory intensive division operations.

  bool enabled = false; // initialization of enable variable
  bool alarmO = false; // initialization of alarm variable
  bool eStop = false; // stops all servo motion until reboot

//PIDF Variables
  // Values must be manually tuned for every specific use case, as they depend on load, inertia, and a few other variables. 
  // Enable debugging serial plotter to assist in tuning (slight performance penalty). Send external stp-dir signals for tuning. Tune for full range of velocities and accelerations expected for best results.
  const double kP = .005; // proportional gain, primary driver behind the PID
  const double kI = .0000000001; // integral gain, used to fine tune results to land on target
  const double kD = .00005; // derivative gain, used to dampen/prevent oscillations and overshoot. 
  const double kF = .0; // Feed Forward gain, used to smooth output change. Can cause issues with sudden direction changes if too large
  const double OutputLim = 0.6; // Absolute output value, use to control power output. Start with low outputs and ramp up as needed

  double pidOUT = 0; // Output storage variable
  double error = 0.0; // Error stored globally to be accessed by encoder and step interrupts
  double lError = 0.0; // last loop position, used to determine velocity for profiled PID mode and for D gain calculation
  int Itimer = 0; // Timer for I gain calculation 


 
  int iThr = 2;
  
//Debugging settings
  #define serialDebugging true // enables serial debugging for serial plotter/monitor. Causes some performance penalty, so only recommended for tuning.
  #define serialBaud 9600 // Higher baudrates result in better performance, but some MCU's don't support certain higher speeds
  #define datarate 100 // Only send data per this number of code loops, reduces performance penalty at the cost of slower data speed
  int counter = 0;
  int debugT = 0;
  const double invDR = 1/datarate;
  //Debug outputs Only enable debugging information that you currently need for best performance. It is suggested to only plot debug data of the same value scale at the same time, otherwise some of the data will be impossible to see on the graphs.
    #define errorDebug true // Displays graph/value of error over time (separation between target and position)
    #define PIDDebug true // Displays output graph/value of PID output over time
    #define enDebug false // Displays whether motor is enabled and e-stopped

void setup() {
  if(Pullup == true){
    pinMode(EnA, INPUT_PULLUP); //sets pins to INPUT PULLUP
    pinMode(EnB, INPUT_PULLUP);
    pinMode(Stp, INPUT_PULLUP);
    pinMode(Dir, INPUT_PULLUP);
    pinMode(En, INPUT_PULLUP); 
    }else{
    pinMode(EnA, INPUT); //sets pins to INPUT PULLUP
    pinMode(EnB, INPUT);
    pinMode(Stp, INPUT);
    pinMode(Dir, INPUT);
    pinMode(En, INPUT); 
    }
  attachInterrupt(digitalPinToInterrupt(Stp), Step, RISING); // attach hardware interrupt
  attachInterrupt(digitalPinToInterrupt(EnA), Encoder, RISING); // attach hardware interrupt

  pinMode(Alarm, OUTPUT); // Sets alarm pin to digital output
  digitalWrite(Alarm, LOW); // Sets alarm pin to low state. Alarm goes high when error present.
  
  if(serialDebugging == true){ // Start serial if serial debugging enabled
    Serial.begin(serialBaud);
  }

}
void loop() {
  enableCheck(); // check En signal and run alarm logic

  if(outMode == 0){
    motor2Pin(PIDF(), OutA, OutB);
    }else{
    motor1Pin(PIDF(), outPin);
    }



  serialDebug();
 
}
void enableCheck(){
  if(EnableSafe == true){ // Check to see if enable safety is active
    enabled = !digitalRead(En); // Check to see if enabled
    }else{
    enabled = true; // Sets enable to true at all times if no enable required
    }
  if(AlarmCheck() == true){
    enabled = false;
  }
}
bool AlarmCheck(){
  if((abs(error) > AlarmLim) && (AlarmMode == 1)){ // Check if absolute error exceeds alarm limit
    digitalWrite(Alarm, HIGH);
    alarmO = true;
    if(AlarmBehavior == 1){
      eStop = true;}
  } else{
    digitalWrite(Alarm, LOW);
    alarmO = false;
  }
  return eStop; // Return signal to Estop if alarm behavior is set to 1 and alarm is triggered. MCU must be restarted to clear e-stop
}
void Encoder(){
  if(enabled == true){ // check for enable signal, don't increment while disabled. (incrimenting while disabled could lead to sudden unexpected movements or alarm trigger when re-enabled)
    if(InvertEncoder == true){
      if(digitalRead(EnB) == HIGH){
        error--;} // Increment up
        else{
        error++;} // Increment down
    }else{
      if(digitalRead(EnB) == HIGH){
        error++;} // Increment up
        else{
        error--;} // Increment down
    }
  }
}
void Step(){
  if(enabled == true){ // check for enable signal, don't increment while disabled. (incrimenting while disabled could lead to sudden unexpected movements or alarm trigger when re-enabled)
    if(digitalRead(Dir) == HIGH){
      error = error + EpS; // Increment up by conversion ratio
    } else{
      error = error - EpS; // Increment down by conversion ratio
    }
  }
}
double PIDF(){

  
  pidOUT = constrain((pidOUT * kF) + ((error * kP) + (error * kI * (Itimer)) - (( error - lError) * kD)), -OutputLim, OutputLim); // PIDF calculation constrained to output limiter values
    lError = error;

  if(enabled == false){ // disables PID if not enabled
    pidOUT = 0.0;
    Itimer == 0;
    }
  if(abs(error) < iThr){ // Reset I timer when error is zero (may cause issues with systems under constant load)
    Itimer = 0;
    }
  
  Itimer++;
  if(invertMotor == true){
    return -pidOUT;
    }else{
      return pidOUT;
    }
  
}

void motor2Pin(double power, int chA, int chB){
  if(power > 0){
    analogWrite(chA, power * 254);
    analogWrite(chB,0);
  } else{
    analogWrite(chB, power * - 254);
    analogWrite(chA, 0);
  }
}

void motor1Pin(double power, int pOut){
  analogWrite(pOut, (power * 127) + 127);
}

void serialDebug(){
  if((serialDebugging == true) && (counter > datarate)){
    counter = 0;
    //print Zero line
      Serial.print("Zero:");
      Serial.print(0);
      Serial.print(",   ");
    if(errorDebug == true){ // Display PID error data
      Serial.print(",   ");
      Serial.print("Error:");
      Serial.print(error);}
    if(PIDDebug == true){ // Display PID output data
      Serial.print(",   ");
      Serial.print("Pid Output:");
      Serial.print(pidOUT);}
    if(enDebug == true){ // Display Servo Enable/Alarm data
      Serial.print(",   ");
      Serial.print("Enable signal:");
      Serial.print(enabled);
      Serial.print(",   ");
      Serial.print("Estop signal:");
      Serial.print(eStop);
      Serial.print(",   ");
      Serial.print("Alarm signal:");
      Serial.print(alarmO);}
      
    Serial.println();
    }

    counter++;
  
  
}







