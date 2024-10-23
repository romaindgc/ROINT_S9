# Projet ROINT

## Joysticks and motors

### Joysticks 

**Joystick 1**  

Initially, the values are $(X,Y) = (530,500) $ 

* Along X axes : values between 0 and 1023  
* Along Y axes : values between 0 and 1023    

**Joystick 2**  

Initially, the values are $(X,Y) = (?,564) $ 

* Along X axes : values between 0 and 1023  
* Along Y axes : values between 0 and 1023 

**Coordinates :**   
* **$(X,Y) = (0,500)$** : joystick is on the **right**
* **$(X,Y) = (1023,500)$** : joystick is on the **left**  

* **$(X,Y) = (530,0)$** : joystick is on the **top**
* **$(X,Y) = (530,1023)$** : joystick is on the **bottom**  

For more information check on the website [joystick](https://whadda.com/product/xy-joystick-module-2-pcs-wpi315/)  

### Quantization

The motors take as input an integer between 0 and 255. Hence, we have to adjust the values $v \in [0,1023]$ into a number $n \in [0,255]$.  
For that, we use the method *map()*.  

#### Function *Map()* :

**Defintion :** Re-maps a number from one range to another. That is, a value of fromLow would get mapped to toLow, a value of fromHigh to toHigh, values in-between to values in-between, etc.  

```cpp
    map(value, fromLow, fromHigh, toLow, toHigh)
```

* `value` : the number to map  
* `fromLow` : the lower bound of the value’s current range  
* `fromHigh` : the upper bound of the value’s current range  
* `toLow` : the lower bound of the value’s target range  
* `toHigh` : the upper bound of the value’s target range  

**Example**  

```cpp
    /* Map an analog value to 8 bits (0 to 255) */
    void setup() {}

    void loop() {
      int val = analogRead(0);
      val = map(val, 0, 1023, 0, 255);
      analogWrite(9, val);
    }
```

All information comes from [arduino map()](https://docs.arduino.cc/language-reference/en/functions/math/map/) 

### Motors and rotation direction
Hence, with the mapped value, we can deduce the velocity and the rotation direction for one motor.  
For controling one motor, we use only the vertical direction of the joystick *ie* the $Y$ axes.  
So we have : 

$$
Rotationdirection = \begin{cases} 
   Counter clockwise & \text{if } Y < 500 \\
   Clockwise & \text{if } Y > 500 
\end{cases}
$$

```cpp
void rotationDirection(int y, int Y_joy, int PIN_SENS){
  if (y < Y_joy ){
    digitalWrite(PIN_SENS, LOW); //Counter clockwise rotation direction
  }else{
    digitalWrite(PIN_SENS, HIGH); //Clockwise rotation direction
  }
}
```
* `y` : is the current position along the y axes of the joystick 
* `Y_joy` : is the inital value of Y of the joystick  
* `PIN_SENS` : is the PIN used to control the rotation direction of a motor  

### Deadzone

The joystick is not perfect. Hence, it never fully returns to its original position. i.e., when you release the joystick, it will not always return to $(X,Y) = (530,500)$.
As a result, a wheel could continue to rotate even if it is not desired.
To avoid this, we create a deadzone: if the value of $Y$ is within the interval $[\alpha, \beta]$, the return value is set to 0. This is known as a **threshold function**. 

```cpp
int thresholdFunction(int x, int lowerBound, int upperBound) {
  if (x > lowerBound) {
    return 0;  // If x is greater than the lowerBound return 0
  } else if (x < upperBound) {
    return 0;  // If x is lower than the upperbound return 0
  } else {
    return 1;  // Else return 1
  }
}
```
We have to adjust the boundaries of the deadzone for each joystick because they do not have the same initial position. 
For example is that case, we choose a deadzone of 10%  of the full range for each joystick :  
```cpp
//Initial value of Y for each joysticks
const int Y_joy1 = 500;
const int Y_joy2 = 564;

//Boundaries for the threshold function
const int upperBound_joy1 = Y_joy1 + ceil(1023*0.01);
const int lowerBound_joy1 = Y_joy1 - ceil(1023*0.01);

const int upperBound_joy2 = Y_joy2 + ceil(1023*0.01);
const int lowerBound_joy2 = Y_joy2 - ceil(1023*0.01);
```

## Function *millis()*

**Description** : Returns the number of milliseconds passed since the Arduino board began running the current program. This number will overflow (go back to zero), after approximately 50 days.  

In every function, we find the following structure :  
```cpp
    unsigned long currentMillis = millis();  
    const unsigned long period = 500;  

    static unsigned long previousMillis = 0;
    if(currentMillis - previousMillis >= period) {
    previousMillis = currentMillis;
    /.../}
```

This structure allows us to choose when we want to execute the methode (fix period) without stopping all the program (contrary to the *delay()* function). Let's introduce all the parameters :  

* `currentMillis` = corresponds to the current time since the program has started.  
* `previousMillis` = refers to the last time the function was called.  
* `period` = corresponds to the choosen period (here 500ms).  

Hence, by taking the difference between *currentMillis* and *previousMillis*, we know the time that has elapsed since the last iteration of the function. If this time is greater than the period, we execute the function.  

**What is the difference with the *delay* function**  

The *delay* function is stopping all the program during a choosen value :
```cpp
    delay(1000)
```
*Exemple : stop the program during 1000ms*  

This function is not suitable for our case because we do not want to stop the process because in the mean time, the code is used for controling the robot, the lights and the camera.    

For more information check on the website [arduino.cc/millis](https://docs.arduino.cc/language-reference/en/functions/time/millis/)  
