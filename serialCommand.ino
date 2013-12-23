
#include <Servo.h>

// INITIALIZE PINS
int panPin = ;
int tiltPin = ;
int loadPin = ;
int hatchPin = ;
int rangePin = ;
int potPin = ;


// other variables
int theta;
int phi;
int delayTime = 70; // milliseconds
int rawRangeData;
int rawPotData;
// also servos
Servo pan;
Servo tilt;
Servo load;
Servo hatch;

// *** SETUP ***


void setup() {
  // initialize serial port
  Serial.begin(9600);

  //Setup Up Launch Sprinkler Valve, Range Finder and Potentiometer
  pinMode(launchValve, OUTPUT); 
  pinMode(rangePin, INPUT);
  pinMode(potpin, INPUT); 


  // setup servos, and center them
  pan.attach(panPin);
  tilt.attach(tiltPin);
  load.attach(loadPin);
  hatch.attach(hatchPin)
  pan.writeMicroseconds(1500);
  tilt.writeMicroseconds(1500);
}


// *** LOOP ***


void loop() {
  // Overall program flow:
  // Is there a command (8 bytes) waiting?
  //First byte = Confirm a new command
  //Second, Third byte = Pan Command
  //Fourth, Fifth byte = Tilt Command
  //Sixth byte = Load Marshmallow
  //Seventh byte = Close Hatch
  //Eight byte = Launch

//Set SprinklerValve
digitalWrite(launchValve, LOW); 

  // If there are at least 8 characters, then we have a command waiting!
  if (Serial.available() >= 8) {

    // Confirm that we're at the start of a new command
    if (Serial.read() == 'a')
    {
      // Next 2 bytes are theta of where to pan to.
      // We take in the number of microseconds to send to the servo (700-2300).
      // The calculating of how many microseconds for a given angle
      // is done on the Python side.
      // We used a big-endian system, so the first byte is *256.
      theta = 0;
      theta += (Serial.read()) * 256;
      theta += (Serial.read()) ;

      // Next 2 digits are phi or what the desired tilt is.
      // Again, first byte is *256.
      phi = 0;
      phi += (Serial.read()) * 256;
      phi += (Serial.read());

      // Move them servos.
      gotoPosition(theta, phi);
      delay(delayTime); // wait for the servos to move

      // Should I load? If yes, do it.
      checkLoad = Serial.read() 
      if (checkLoad == )
      {
        runLoad()
      }

      // Should I move the hatch? If yes, do it.
      checkHatch = Serial.read() 
      if (checkHatch == )
      {
        runHatch()
      }

      // Should I Fire(Launch)? If yes, do it.
      checkLauch = Serial.read() 
      if (checkLaunch == )
      { 
        runLaunch()
      }

      rawRangeData = readData(rangePin);
      rawPotData = readData(potPin);
      sendData(rawRangeData, rawPotData);

    }
  }
}


// *** OTHER FUNCTIONS ***

// read the data 
int readData(pin) { 
  return analogRead(pin);
}


void sendData (int rawRangeData, int rawPotData) {

  // First, an id that we're at the start of a command
  Serial.write('o');
  Serial.write(rawRangeData / 256); // highByte range finder data
  Serial.write(rawRangeData % 256); // lowByte range finder data

  Serial.write(rawPotData / 256); // highByte pot data
  Serial.write(rawPotData % 256); // lowByte pot data

void runLoad()
{
//Write loading microseconds
//Drop
load.writeMicroseconds()
//Wait For marshmallow to fall
delay(2000)
//Catch next
load.writeMicroseconds()
}

void runHatch()
{
//Write hatch microseconds
hatch.writeMicroseconds()
}

void runLaunch()
{
//Release the Valve, Launch 
digitalWrite(launchValve, HIGH); 
delay(2000);
digitalWrite(launchValve, LOW); 
}

void gotoPosition(int theta, int phi)
{
  // Writes the # of microseconds to each servo.
  pan.writeMicroseconds(theta);
  tilt.writeMicroseconds(phi);
}
