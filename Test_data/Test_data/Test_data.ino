//Libraries
#include <Wire.h>
#include <SoftwareSerial.h>
#include "Servo.h"
#include <I2Cdev.h>


//Définition des variables pour le bluetooth
const int RX = 11;
const int TX = 10;
SoftwareSerial bluetoothSerial(RX, TX); // RX, TX


void setup() {
  Serial.begin(9600);

 // define pin modes for tx, rx pins:
  pinMode(RX, INPUT);
  pinMode(TX, OUTPUT);
  bluetoothSerial.begin(9600);
}

void loop() {

  //Lecture  et traitement de la data reçue par bluetooth
  //Format XXvalue!
  if (bluetoothSerial.available()) {
    Serial.println("Available");
    
    // Lire la chaîne jusqu'au caractère '!'
    String data = bluetoothSerial.readStringUntil('!');

    Serial.println("data reçue : ");
    Serial.println(data);

    // Vérifier que la chaîne contient au moins 2 lettres suivies d'une valeur
    if (data.length() >= 3) {
      // Extraire les deux lettres
      String prefix = data.substring(0, 2);

      // Extraire la valeur entière après les deux lettres
      String valueString = data.substring(2); // À partir du troisième caractère
      int value = valueString.toInt(); // Convertir en entier

      // Afficher les résultats pour débogage
      Serial.print("Lettres reçues : ");
      Serial.println(prefix);
      Serial.print("Valeur reçue : ");
      Serial.println(value);

    }
  }
  
}
