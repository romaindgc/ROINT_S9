

int VRx = A0; // VRx to analog input 0
int VRy = A1; // VRy to analog input 1
int SW = 2; // SW to digital pin 2
int xPosition = 0; // Initialize xPosition variable with 0
int yPosition = 0; // Initialize yPosition variable with 0
int SW_state = 0; // Initialize SW_state variable with 0


void setup() {
Serial.begin(9600); // Initialize Serial connection with a baudrate of 9600 bits/s
pinMode(VRx, INPUT); // Set VRx pin as INPUT
pinMode(VRy, INPUT); // Set VRy pin as INPUT
pinMode(SW, INPUT_PULLUP); // Enable internal pullup resistor on switch pin
}


void loop() {
xPosition = analogRead(VRx); // Read x position of joystick
yPosition = analogRead(VRy); // Read y position of joystick
SW_state = digitalRead(SW); // Read switch state
Serial.print(xPosition); // Print x position of joystick (0->1023)
Serial.print("\t"); // Add tab character to seperate the 2 values
Serial.println(yPosition); // Print y position of joystick (0->1023)
delay(10);
}
