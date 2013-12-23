#include <Servo.h>

//Pin Variables
int tiltPin = 11;  //Yellow signal
int panPin = 5;
int solenoidPin = 9;
int sealPin = 10;  //Blue signal
int loaderPin = 3;  //Green signal
int potPin = 5;

//Angle Variables
int rawData = 0;
int angle = 0;
int angleInt = 0;
int pangle = 0;
int baseAngle = -13;
int lastPot = 150;

//Control Variables
int difference = 0;
int panSetpoint = 150;
int panSignal = 0;
int panSpeed = 7;
int tiltCommand = 13;
long lastSent = 0;
int stopVal = 80;
int panCommand = stopVal;
char commandType;
int whichWay = 1;

Servo pan;
Servo tilt;
Servo seal;
Servo loader;


void setup(){
  Serial.begin(38400);
  delay(2000);
  Serial.flush();
  
  pan.attach(panPin);
  tilt.attach(tiltPin);
  seal.attach(sealPin);
  loader.attach(loaderPin);
  
  pinMode(solenoidPin, OUTPUT); 

  pan.write(stopVal);
  tilt.write(45 * 3 - baseAngle);
 
 
 
}




void loop(){
  
  
  
  
  
  pangle = getAngle();
  //Serial.println(String(pangle));
  if(Serial.available() >= 3){
    
    commandType = Serial.read();
    if (commandType == 'p'){
      panCommand = 0;
      panCommand += (Serial.read()) * 256;
      panCommand += (Serial.read());
      panCommand = panCommand - 40;
      panSetpoint = -panCommand + lastPot;
      lastSent = millis();
    }
    else if(commandType == 'g'){
      Serial.read();
      Serial.read();
      sendData(Serial.available());
    }
    else if(commandType == 'c'){
      lastPot = getAngle();
      Serial.read();
      Serial.read();
    }
    
    else if(commandType == 't'){
      tiltCommand = 0;
      tiltCommand += (Serial.read()) * 256;
      tiltCommand += (Serial.read()); 
      lastSent = millis();
    }
    
    else if(commandType == 'f'){
      fireMarshmallow();
      Serial.read();
      Serial.read();
    }

    else if(commandType == 'h'){
      seal.writeMicroseconds(200);
      Serial.read();
      Serial.read();
    }

    else if(commandType == 'r'){
      reload();
      Serial.read();
      Serial.read();
    }
    
    else if(commandType == 's'){
      seal.writeMicroseconds(2070);
      Serial.read();
      Serial.read();
    }
  }
  
  if ((pangle > 270) || (pangle < 30)){
    center();
  }
  
  else if((millis() - lastSent) > 1000000){
    randomPan();
  }
  
  else{
    difference = pangle - panSetpoint + 0 * difference;
    panSignal = 1.2 * difference + stopVal;
    pan.write(panSignal);
    tilt.write(tiltCommand);
  }
}

void fireMarshmallow(){     
  pan.write(stopVal);
  seal.writeMicroseconds(2100);  // Pre-load seal prior to firing
  delay(350);
  digitalWrite(solenoidPin, HIGH);  // Open solenoid valve to fire
  delay(200);
  digitalWrite(solenoidPin, LOW);
  seal.writeMicroseconds(500);  // Open seal to reload
  delay(180);
  loader.writeMicroseconds(1300);  // Drop single marshmallow
  delay(300);
  loader.writeMicroseconds(1900);  // Prepare to drop another marshmallow
  delay(200);
  seal.writeMicroseconds(2070);  // Close seal
  delay(100);
  pan.write(stopVal);
}

void reload(){
  pan.write(stopVal);
  seal.writeMicroseconds(500);  // Open seal to reload
  delay(180);
  loader.writeMicroseconds(1300);  // Drop single marshmallow
  delay(300);
  loader.writeMicroseconds(1900);  // Prepare to drop another marshmallow
  delay(200);
  seal.writeMicroseconds(2070);  // Close seal
  delay(100);
  pan.write(stopVal);
}


int getAngle(){
  rawData = analogRead(potPin) + 100;
  angle =(rawData) * 300.0 / 1024;
  angleInt = angle - 150;
  return angle;
}


void center(){
  while(abs(getAngle() - 150) > 5){
    panCommand = (getAngle() - 150) + stopVal;
    pan.write(panCommand);
  
  }
}

void sendData(int angleOut){
  Serial.print('a');
  Serial.write(angleOut/256);
  Serial.write(angleOut%256);
}

void randomPan(){
  tilt.write(45*3 - baseAngle);
  pan.write(stopVal + (whichWay * panSpeed));
  if (angle < 50){
    whichWay = -1;
  }
  else if (angle > 250){
      whichWay = 1;
  }
  
}
  
