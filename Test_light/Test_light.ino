//Libraries
#include <Wire.h>
#include <SoftwareSerial.h>
#include "Servo.h"
#include <I2Cdev.h>

//Définition des variables pour le système d'éclairage
const int PIN_ENABLE_LEDS = A3; //PIN pour la commande de l'éclairage
int CHOIX_MODE = 10; // 1000 = éclairage --  2000 = defense
int CHOIX_MODE_PREVIOUS = 10;
const int frequence_clignotement = 7; //En ms


//Définition des variables pour le bluetooth
//const int RX = 1;
//const int TX = 0;
//SoftwareSerial bluetoothSerial(RX, TX); // RX, TX


void setup() {
  Serial.begin(9600);

  //Définition 
  pinMode (PIN_ENABLE_LEDS, OUTPUT); //définition du PIN ENABLE LED comme une sortie
  analogWrite(PIN_ENABLE_LEDS, 0); //on éteind l'éclairage par défaut

 // // define pin modes for tx, rx pins:
 // pinMode(RX, INPUT);
 // pinMode(TX, OUTPUT);
 // bluetoothSerial.begin(38400);
}

void loop() {
  //readMPU();
  //readTension();
  //CHOIX_MODE = (int)bluetoothSerial.read(); //Lecture de la commande bluetooth

  String data = Serial.readStringUntil('\n'); // Lire jusqu'à la fin de ligne
  int CHOIX_MODE = data.toInt(); // Convertir en entier
  Serial.println("Choix reçu : ");
  Serial.println(CHOIX_MODE); // Afficher le nombre reçu

  if (CHOIX_MODE != 10 && CHOIX_MODE != 20 && CHOIX_MODE != 30 ){
      Serial.println("Choix precedent ");
      CHOIX_MODE = CHOIX_MODE_PREVIOUS;
  }else{
    Serial.println("New choice ");
    CHOIX_MODE_PREVIOUS = CHOIX_MODE;
  }

  //testservo();
  choixModeEclairage(CHOIX_MODE);
}


//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : CHOIX MODE ECLAIRAGE
//-----------------------------------------------------------------------------------------------------------------
  void choixModeEclairage(int choix){
    switch (choix) {
    case 10:
        eclairageEteint();
        break;

    case 20:
        eclairageAllume();
        break;

    case 30:
        eclairageDefense();
        break;

    default:
        eclairageEteint();
        break;
}

  }
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Système d'éclairage -- MODE ETEINT -- 0000
//-----------------------------------------------------------------------------------------------------------------
  void eclairageEteint(){
    analogWrite(PIN_ENABLE_LEDS,0); //On éteind les LEDs
    Serial.println("LEDs eteintes");
  }
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Système d'éclairage -- MODE ECLAIRAGE -- 1000
//-----------------------------------------------------------------------------------------------------------------
 void eclairageAllume(){
    analogWrite(PIN_ENABLE_LEDS,1023); //Allumage des LEDs
    Serial.println("LEDs allumees");
 }
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Système d'éclairage -- MODE DEFENSE -- 2000
//-----------------------------------------------------------------------------------------------------------------
void eclairageDefense(){
  Serial.println("Mode defense active");
  analogWrite(PIN_ENABLE_LEDS,1023); //Allumage des LEDs
  delay(frequence_clignotement);
  analogWrite(PIN_ENABLE_LEDS,0); //LEDs éteintes
  delay(frequence_clignotement);
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
