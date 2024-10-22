//Libraries
#include <Wire.h>
#include <SoftwareSerial.h>
#include "Servo.h"
#include <I2Cdev.h>

Servo servo; // création de l'objet "servo"
const int servoPin = 5; //définition du pin du servo
const int JoyStickPin = A5;

//Définition variable pour la lecture angle y
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int minVal=265;
int maxVal=402;
double x;
double y;
double z;

//Définition des variables pour le système d'éclairage
const int PIN_ENABLE_LEDS = 3; //PIN pour la commande de l'éclairage
int CHOIX_MODE = 0000; // 1000 = éclairage --  2000 = defense
const int frequence_clignotement = 50; //En ms

//Définition variable pour asservissement et correcteur PID
const double Consigne = 0.0; //consigne à atteindre
double prevError = 0.0;
double I = 0.0;
const double Kp = 2.0;
const double Ki = 0.1;
const double Kd = 1.0;
int mode_camera = 21; //Mode automatique = 21 - Mode manuel = 31 |

//Définition des variables pour le bluetooth
const int RX = 1;
const int TX = 0;
SoftwareSerial bluetoothSerial(RX, TX); // RX, TX

//Définition des périodes et variables de temps
unsigned long currentMillis = millis();   // Temps actuel
const unsigned long periodServo = 500;         // Intervalle de 500 ms
const unsigned long periodGyro = 500;
const unsigned long periodTension = 5000;

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);

  //Définition servo
  servo.attach(servoPin); // attache le servo au pin spécifié
  servo.write(Consigne); //définir la position initiale du servo

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
  //CHOIX_MODE = (int)bluetoothSerial.read(); //Lecture de la commande bluetooth
  //testservo();
  //choixModeEclairage(CHOIX_MODE);
  int ValueY = analogRead(JoyStickPin);
  Serial.println(ValueY);
  choixModeCamera(31, ValueY);
  delay(1000);
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

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : CHOIX MODE CAMERA
//-----------------------------------------------------------------------------------------------------------------
  void choixModeCamera(int choix, int JSValueY){
    switch (choix) {
    case 21:
        ModeAutoCamera();
        break;

    case 31:
        ModeLibreCamera(JSValueY); 
        break;

    default:
        //Mode auto par défaut
        break;
}

  }
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Mode automatique camera
//-----------------------------------------------------------------------------------------------------------------
  void ModeAutoCamera(){
    double angle_y=lire_angle_gyro(); //définir l'angle y comme la lecture de l'angle avec le gyro
    asservissement_servo(angle_y);
  }
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Mode libre camera
//-----------------------------------------------------------------------------------------------------------------
  void ModeLibreCamera(int JSValueY) {
    int angleY; 
    
    // Define the dead zone limits (10% from the top and bottom)
    int deadZoneLow = 530-0.1*1023;  // 530 -> center of the joytstick
    int deadZoneHigh = 530+0.1*1023; //
    Serial.print("Value ="); 
    Serial.println(JSValueY);
    if (JSValueY > deadZoneLow && JSValueY < deadZoneHigh) {
      //asservissement_servo(0);
      Serial.print("Value angle ="); 
      Serial.println(90);
      servo.write(90);
      return;
    }
    else {
      // Map the joystick value (0-1023) to the servo angle (0-180)
      angleY = map(JSValueY, 0, 1023, 0, 180);
      Serial.print("Value angle ="); 
      Serial.println(angleY);
      servo.write(angleY);
      
      //asservissement_servo(angleY);
    }    
  }
