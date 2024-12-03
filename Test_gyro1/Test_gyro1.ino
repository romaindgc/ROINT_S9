//Libraries
#include <Wire.h>
#include <SoftwareSerial.h>
#include "Servo.h"
#include <I2Cdev.h>

Servo servo; // création de l'objet "servo"
const int servoPin = 5; //définition du pin du servo


//Définition variable pour la lecture angle y
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int minVal=265;
int maxVal=402;
double x;
double y;
double z;


//Définition des variables pour le bluetooth
//const int RX = 1;
//const int TX = 0;
//SoftwareSerial bluetoothSerial(RX, TX); // RX, TX

//Définition des périodes et variables de temps
unsigned long currentMillis = millis();   // Temps actuel
const unsigned long periodServo = 500;         // Intervalle de 500 ms
const unsigned long periodGyro = 500;
const unsigned long periodTension = 5000;

void setup() {
  //Setup gyroscope
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  //Définition servo
  servo.attach(servoPin); // attache le servo au pin spécifié
  servo.write(Consigne); //définir la position initiale du servo

  Serial.begin(9600);


  //Définition 
  pinMode (PIN_ENABLE_LEDS, OUTPUT); //définition du PIN ENABLE LED comme une sortie
  analogWrite(PIN_ENABLE_LEDS, 0); //on éteind l'éclairage par défaut

  // define pin modes for tx, rx pins:
  pinMode(RX, INPUT);
  pinMode(TX, OUTPUT);
  bluetoothSerial.begin(38400);
}

void loop() {
  //readMPU();
  //readTension();
  CHOIX_MODE = (int)bluetoothSerial.read(); //Lecture de la commande bluetooth
  //testservo();
  choixModeEclairage(CHOIX_MODE);
  choixModeCamera(mode_camera);
}


//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Asservissement servomoteur
//-----------------------------------------------------------------------------------------------------------------
void asservissement_servo(double angleY){
  static unsigned long previousMillis = 0;
  if(currentMillis - previousMillis >= periodServo) {
    previousMillis = currentMillis;
         // Calcul de l'erreur
    double error = Consigne - angleY;
    Serial.print("Error = ");
    Serial.println(error);
    // Calcul des termes PID
    double P = Kp * error;
    if (-3 < error < 3) {
    I += Ki * error;
    }
    double D = Kd * (error - prevError)/periodServo;
  
    // Calcul de la commande du servomoteur
    double servoCommand = P + I + D;
    servoCommand = map(servoCommand, -150, 150, -50, 50);
    Serial.print("servoCommand = ");
    Serial.println(servoCommand);
    // Limiter la commande du servomoteur dans la plage acceptable
    //servoCommand = constrain(servoCommand, 0, 180);
  
    // Envoyer la commande au servomoteur
    servoCommand = servoCommand+angleY;
    Serial.print("servoCommand = ");
    Serial.println(servoCommand);
    servo.write(servoCommand);
  
    // Mise à jour de l'erreur précédente
    prevError = error;
  }
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------



//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Test du servomoteur
//-----------------------------------------------------------------------------------------------------------------
void testservo() {
   servo.write(0); // demande au servo de se déplacer à cette position
   delay(1000); // attend 1000 ms entre changement de position

   servo.write(90); // demande au servo de se déplacer à cette position
   delay(1000); // attend 1000 ms entre changement de position

   servo.write(180); // demande au servo de se déplacer à cette position
   delay(1000); // attend 1000 ms entre changement de position
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------



//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Capter la position avec le gyroscope
//-----------------------------------------------------------------------------------------------------------------
double lire_angle_gyro( ) { 
  //a vérifier
  static unsigned long previousMillis = 0;
  if(currentMillis - previousMillis >= periodGyro) {
    previousMillis = currentMillis;
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
  
    //lecture des angles
    AcX=Wire.read()<<8|Wire.read();
    AcY=Wire.read()<<8|Wire.read();
    AcZ=Wire.read()<<8|Wire.read();
    int xAng = map(AcX,minVal,maxVal,-90,90);
    int yAng = map(AcY,minVal,maxVal,-90,90);
    int zAng = map(AcZ,minVal,maxVal,-90,90);
  
    //conversion des angles en degrés
    x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
    y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
    z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
     
    //renvoi de la valeur de l'angle y
    Serial.println("AngleY= ");
    Serial.print(y);
    Serial.println("-----------------------------------------");
  
    return y;
  }
  return -1;
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
