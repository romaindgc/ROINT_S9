#include <math.h> 

// PIN MOTEURS
const int PIN_M1 = 3;    // Port D3 PWM
const int PIN_M2 = 5;    // Port D5 PWM

const int PIN_SENS_M1 = A2; // Port A2  On peut utiliser les ports analogiques comme des digitaux
const int PIN_SENS_M2 = A1; // Port A1

int value_M1;
int value_M2;

//For testing we put the joytsiks on A3 and A4
const int PIN_JOY1 = A3;
const int PIN_JOY2 = A4;

//Initial value of Y for each joysticks
const int Y_joy1 = 500;
const int Y_joy2 = 564;

//Boundaries for the treshold function
const int upperBound_joy1 = Y_joy1 + ceil(1023*0.01);
const int lowerBound_joy1 = Y_joy1 - ceil(1023*0.01);

const int upperBound_joy2 = Y_joy2 + ceil(1023*0.01);
const int lowerBound_joy2 = Y_joy2 - ceil(1023*0.01);


  void setup() {
  // Initialisation de la connexion série à un débit de 9600 bits/s
  Serial.begin(9600);

  // Configuration des PINS comme des sorties
  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_M2, OUTPUT);
  pinMode(PIN_SENS_M1, OUTPUT);
  pinMode(PIN_SENS_M2, OUTPUT);

  pinMode(PIN_JOY1, INPUT);
  pinMode(PIN_JOY2, INPUT);

  // Setup initial des moteurs
  setupMotors();
}

  void loop() {

  //Read the input values from joysticks
  value_M1 = analogRead(PIN_JOY1);
  value_M2 = analogRead(PIN_JOY2);

  //Apply the treshold function
  value_M1 = value_M1*thresholdFunction(value_M1,lowerBound_joy1,upperBound_joy1);
  value_M2 = value_M2*thresholdFunction(value_M2,lowerBound_joy2,upperBound_joy2);

  //Mapping the input values from joysticks to fit with the input of the motors
  value_M1 = map(value_M1, 0, 1023, 0, 255);
  value_M2 = map(value_M2, 0, 1023, 0, 255);
  Serial.print("value mapped M1 : ");
  Serial.println(value_M1);
  Serial.print("--------------------");
  Serial.print("value mapped M2 : ");
  Serial.println(value_M2);

}

//-------------------------------------Method : Configuration motors --------------------------
void setupMotors() {
  // Vitesse de rotation des moteurs nulle
  analogWrite(PIN_M1, 0);
  analogWrite(PIN_M2, 0);

  // Sens de rotation par défaut : vers l'avant
  digitalWrite(PIN_SENS_M1, HIGH);
  digitalWrite(PIN_SENS_M2, HIGH);
}
//--------------------------------------------------------------------------------------------------

//-------------------------------------Method : Command motors --------------------------
void commandMotors(int value_mapped_M1, int value_mapped_M2) {
  // Vitesse de rotation des moteurs nulle
  analogWrite(PIN_M1, value_mapped_M1);
  analogWrite(PIN_M2, value_mapped_M2);

  // Sens de rotation par défaut : vers l'avant
  rotationDirection(value_mapped_M1, Y_joy1, PIN_M1);
  rotationDirection(value_mapped_M2, Y_joy2, PIN_M2);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------- Treshold function ----------------------------------------------
  int thresholdFunction(int x, int lowerBound, int upperBound) {
  if (x > lowerBound) {
    return 0;  // If x is greater than the lowerBound return 0
  } else if (x < upperBound) {
    return 0;  // If x is lower than the upperbound return 0
  } else {
    return 1;  // Else return 1
  }
}
//--------------------------------------------------------------------------------------------------

//------------------------------Method : Rotation direction-----------------------------------------
void rotationDirection(int y, int Y_joy, int PIN_SENS){
  if (y < Y_joy ){
    digitalWrite(PIN_SENS, LOW); //Counter clockwise rotation direction
  }else{
    digitalWrite(PIN_SENS, HIGH); //Clockwise rotation direction
  }
}
