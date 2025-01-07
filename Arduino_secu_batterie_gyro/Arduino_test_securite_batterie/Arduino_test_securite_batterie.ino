#include <SoftwareSerial.h>


void setup() {
  Serial.begin(9600); // Communication série avec le moniteur série de l'ATMega
  pinMode(A0, INPUT); // Configure le pin A0 en entrée
  pinMode(4, OUTPUT); // Configure la broche 4 en tant que sortie

}

void loop() {
  readTension();
}

void readTension( ) {
  digitalWrite(4, HIGH); // Définit la broche 4 en état HIGH
  delay(100);
  digitalWrite(4, LOW); // Définit la broche 4 en état LOW
  float sommeTension = 0.0; //définition pour faire la moyenne de la tension
  int echantillon=1000; //A MODIFIER, valeur du nombre d'echantillon qui définie la précision de notre mesure
  for (int i = 0; i < echantillon; i++) { //on récupère toutes les valeurs selon le nombre d'échantillon désiré
    int valeurTension = analogRead(A0);
    float tension = (float)valeurTension * 5.0 / 1023.0; //on mutlplie par 5/1023 pour obtenir la valeur en tension, l'arduino étant codé sur 1024 bits
    sommeTension += tension; 
    
    delay(100/echantillon); // Attendre un peu entre chaque lecture en fonction du nombre d'échantillon (10 secondes)
  }

  // Calculer la moyenne des échantillons
  float moyenneTension = sommeTension / echantillon;

  // Envoi de la moyenne via Bluetooth
  Serial.print("Moyenne tension (");
  Serial.print(echantillon);
  Serial.print(" echantillons): ");
  Serial.print(moyenneTension, 2); // Affiche la moyenne avec 2 décimales
  Serial.println(" V");

  delay(100); // Attendez  0.1 seconde avant de lire à nouveau la valeur
}
