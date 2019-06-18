#define MAX_TEMP  300
#define MAX_SPEED 300
#define TIMOUT    5000

#define CHA        0
#define CHB        1
#define BUTTON     7
#define TRIGGER    8
#define HEATER     5
#define MOTOR      6
#define DIR_INPUT  9
#define DIR_OUTPUT 16
#define ENABLE     14
#define FORWARD    1
#define REVERSE    -1

#define FILAMENT_DIAM       3.0  //mm
#define NOZZLE_DIAM         0.5  //mm
#define GEAR_DIAM           10.8 //mm
#define STEP_SIZE           16    //4 is 1/4 step 8 is 1/8 step
#define STEPS_PER_ROTATION  200  

//FYI Pin 10 is dead!! Don't use!!

// which analog pin to connect
#define THERMISTORPIN A0         
// resistance at 25 degrees C
#define THERMISTORNOMINAL 100000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 3800    

//PI control values
#define Kp  1.48134292984995
#define Ki  0.0131800567660014
#define Kd  18.6947422906608
#define Kf  2.0
 
uint16_t samples[NUMSAMPLES];

byte upArrow[8] = {    //Create up arrow for display
  B00100,
  B01110,
  B11111,
  B00100,
  B00100,
  B00100,
  B00100,
};
byte downArrow[8] = {    //Create down arrow for display
  B00100,
  B00100,
  B00100,
  B00100,
  B11111,
  B01110,
  B00100,
};
#include <LiquidCrystal_I2C.h>
#include <Stepper.h>
#include <math.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);   // set the LCD address to 0x27 for a 16 chars and 2 line display
//Stepper motor(200, 5, 6, 8, 9);

volatile int master_count = 0; // universal count
volatile byte INTFIRED = 0; // interrupt status flag
volatile int dispMenu = 0;  //which menu to display

int setTemp = 100;
float temperature = 20.0;
int temp, nextTemp;
int feedRate = 10;
boolean driveFlag = false;

void setup() { 
   Serial.begin(9600);
   lcd.begin(16,2);                      // initialize the lcd 
   lcd.createChar(0, upArrow);          //Creat arrows for display
   lcd.createChar(1, downArrow);
  lcd.backlight();
  pinMode(CHA, INPUT);
  pinMode(CHB, INPUT);
  pinMode(BUTTON, INPUT);
  pinMode(TRIGGER, INPUT_PULLUP);
  pinMode(MOTOR, OUTPUT);
  pinMode(HEATER, OUTPUT);
  digitalWrite(HEATER,LOW);
  pinMode(DIR_INPUT, INPUT_PULLUP);
  pinMode(DIR_OUTPUT,OUTPUT);
  pinMode(ENABLE, OUTPUT);
  //motor.setSpeed(long(feedRate/12.0/3.14*60));
  
  attachInterrupt(2, flag, RISING);// interrupt 0 digital pin 2 positive edge trigger
  attachInterrupt(3, flag2, RISING);  
  attachInterrupt(digitalPinToInterrupt(BUTTON), buttonFlag, FALLING);
  updateDisplay();
  //analogReference(EXTERNAL);

  lcd.clear();
  lcd.print("Temp:"+String(temperature));
  lcd.setCursor(8,0);
  lcd.print(" Set:"+String(setTemp));
  lcd.setCursor(0,1);
  lcd.print("Feed Rate:" + String(feedRate));
  lcd.cursor();
}

//Main Control Loop
void loop() {
  controlTemp((float) (temperature=checkTemp()));
  temp = round(temperature);
  if (INTFIRED)   {
       updateDisplay();
     INTFIRED = 0; // clear flag
     nextTemp = temp;
  } 
  else if(nextTemp!=temp){
    updateDisplay();
    nextTemp = temp;
  }
  
if(digitalRead(TRIGGER)!=driveFlag){
  drive(driveFlag);
  driveFlag = !driveFlag;
}

} // end loop

void drive(boolean dr){
  
  if(dr){
    if(true){//dispMenu == 0){  //Only drive motor if not editing temp or speed
      tone(MOTOR, feedRate2PulseFreq(feedRate));
      lcd.setCursor(14,1);
      if(digitalRead(DIR_INPUT)){
        digitalWrite(DIR_OUTPUT,HIGH);
        lcd.write(byte(0));
      }
      else{
        digitalWrite(DIR_OUTPUT,LOW);
        lcd.write(byte(1));
      }
      digitalWrite(ENABLE,LOW);
    }
  }
  else{
    noTone(MOTOR);
    digitalWrite(ENABLE,HIGH); 
    lcd.setCursor(14,1);
    lcd.print(" ");
    lcd.noCursor();
  }
}

//Function to convert form feedRate in mm/s to pulse frequency to motor in pulses per second
int feedRate2PulseFreq(int feedRate){
  float rps = ((float) feedRate)/(PI*GEAR_DIAM);
  int pulseFreq = (int) (rps*STEP_SIZE*STEPS_PER_ROTATION*NOZZLE_DIAM/FILAMENT_DIAM);
  return pulseFreq;
}

float checkTemp(){
  uint8_t i;
  float average;
 
  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(THERMISTORPIN);
   delay(10);
  }
 
  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;
 
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
 
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
  return steinhart;
}

float error=0, prevError=0;
float totalError;
unsigned long now, lastTime=millis();
unsigned long sent = millis();

//PID temperature control
void controlTemp(float sense){
  float duty;
  error = (setTemp-sense);
  //Serial.println("Error: "+String(error));
 
  now = millis();
  double delt = (double) ((now-lastTime)/1000.0);  //Find change in time (seconds)
   totalError+=(error*delt);
  //Serial.println("Delt: "+String(delt));
  duty = (Kp*error + Ki*totalError + Kd*(error-prevError)/delt + (driveFlag?0:1)*Kf); //The PID formula with an added feed-forward term that contributes to heating when filiment is fed
  //Set Saturation
  if(duty>100){
    duty=100;
  }
  else if(duty<0){
    duty=0;
  }
  lastTime = now;
  prevError = error;
  analogWrite(HEATER,round(duty*2.55));
  if(now-sent>=500){
  Serial.println(String(duty)+','+String(sense));
  sent = now;
  }
}

void updateDisplay(){
  if(dispMenu == 1){        //If temperature is selected
        setTemp += master_count;
        if(setTemp>MAX_TEMP){
          setTemp=MAX_TEMP;
        }
        else if(setTemp<0){
          setTemp=0;
        }
       }
   else if(dispMenu == 2){  //If feedrate is selected
    feedRate += master_count;
    if(feedRate>MAX_SPEED){
      feedRate = MAX_SPEED;
    }
    else if(feedRate<0){
      feedRate = 0;
    }
    
   }
       master_count=0;
       lcd.setCursor(5,0);
       lcd.print("   ");
       lcd.setCursor(5,0);
       lcd.print(temp);
       lcd.setCursor(13,0);
       lcd.print("   ");
       lcd.setCursor(13,0);
       lcd.print(setTemp);
       lcd.setCursor(10,1);
       lcd.print("   ");
       lcd.setCursor(10,1);
       lcd.print(feedRate);
       lcd.cursor();
       if(dispMenu == 1){
        lcd.setCursor(12,0);
       }
       else if(dispMenu == 2){
        lcd.setCursor(9,1);
       }
       else{
        lcd.noCursor();
       }
}

void flag() { //If pin 2 is rising 
  INTFIRED = 1;
  // add 1 to count for CW
  if (digitalRead(CHA) && !digitalRead(CHB)) {
    master_count++ ;
  }
  // subtract 1 from count for CCW
  if (digitalRead(CHA) && digitalRead(CHB)) {
    master_count-- ;
  } 
}
void flag2(){ //If pin 3 is rising 
  INTFIRED = 1;
  //add 1 to count for CW
  if(digitalRead(CHB) && digitalRead(CHA)){
    master_count++;
  }
  //subtract 1 from count for CCW
  if(digitalRead(CHB) && !digitalRead(CHA)){
    master_count--;
  }
}
void buttonFlag(){
  INTFIRED = 1;
  dispMenu = (dispMenu+1)%3;  //Switches between which of the 3 items are selected
}
