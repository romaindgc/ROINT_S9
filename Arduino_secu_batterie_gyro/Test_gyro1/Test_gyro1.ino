//Libraries
#include <Wire.h>//https://www.arduino.cc/en/reference/wire
#include <Adafruit_MPU6050.h>//https://github.com/adafruit/Adafruit_MPU6050
#include <Adafruit_Sensor.h>//https://github.com/adafruit/Adafruit_Sensor
#include <SoftwareSerial.h>

SoftwareSerial bluetoothSerial(1, 0); // RX, TX

//Objects
Adafruit_MPU6050 mpu;

void setup() {
  //Init Serial USB
  Serial.begin(9600);
//  if (!mpu.begin(0x69)) { // Change address if needed
//    Serial.println("Failed to find MPU6050 chip");
//    while (1) {
//      delay(10);
//    }
//  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  readMPU();
  delay(5000);
}


//----------------------------------------Fonction : Capter la position avec le gyro  ---------------------------------
void readMPU( ) { /* function readMPU */
  ////Read acceleromter data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  static float angleX = 0.0;
  static float angleY = 0.0;
  static float angleZ = 0.0;

  float dt = 0.005;  // Intervalle de temps en secondes (ajustez si nécessaire)

  angleX += g.gyro.x * dt;
  angleY += g.gyro.y * dt;
  angleZ += g.gyro.z * dt;

  Serial.println("Rotation X: ");
  Serial.print(angleX);
  Serial.print(", Y: ");
  Serial.print(angleY);
  Serial.print(", Z: ");
  Serial.print( angleZ);
  Serial.println(" rad");

  

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println("Degre Celsius");
}
//-------------------------------------------------------------------------------------------------------------------------

//----------------------------------------Fonction : Lire la tension pour la sécu. batterie ---------------------------------

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
//-------------------------------------------------------------------------------------------------------------------------
