#define POT_PIN A0       //Pin for the potentiometer
#define EN_A 5           //Enable pin for the motor driver
#define IN_1 6           //Input 1 pin for the motor driver
#define IN_2 7           //Input 2 pin for the motor driver
#define OPEN_LIMIT 9     //Pin for the open limit switch
#define CLOSE_LIMIT 8    //Pin for the close limit switch
#define INTERRUPT_PIN 2  //Pin for the interrupt pin (for the pulse analizer)
#define KP 10.0          // Proportional gain
#define KI 5.0           // Integral gain
#define KD 1.5           // Derivative gain

#define SETUP_CPP        // TAG to compile setup.cpp or main.cpp