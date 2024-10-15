# Projet ROINT

## Joysticks

Initially, the values are (X,Y) = (530,500)  

* Along X axes : values between 0 and 1023  
* Along Y axes : values between 0 and 1023  

Coordinates :  
* **(X,Y) = (0,500)** : joystick is on the **right**
* **(X,Y) = (1023,500)** : joystick is on the **left**  

* **(X,Y) = (530,0)** : joystick is on the **top**
* **(X,Y) = (530,1023)** : joystick is on the **bottom**  

For more information check on the website [joystick](https://whadda.com/product/xy-joystick-module-2-pcs-wpi315/)  

## Fonction *millis()*

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

* *currentMillis* = corresponds to the current time since the program has started.  
* *previousMillis* = refers to the last time the function was called.  
* *period* = corresponds to the choosen period (here 500ms).  

Hence, by taking the difference between *currentMillis* and *previousMillis*, we know the time that has elapsed since the last iteration of the function. If this time is greater than the period, we execute the function.  

**What is the difference with the *delay* function**  

The *delay* function is stopping all the program during a choosen value :
```cpp
    delay(1000)
```
*Exemple : stop the program during 1000ms*  

This function is not suitable for our case because we do not want to stop the process because in the mean time, the code is used for controling the robot, the lights and the camera.    

For more information check on the website [arduino.cc/millis](https://docs.arduino.cc/language-reference/en/functions/time/millis/)  
