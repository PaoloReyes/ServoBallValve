#include <constants.h>

#ifdef MAIN_CPP
  #include <valve.h>
  #include <utils.h>

  //Create a valve object and variables for storing the opening and closing limits
  Valve valve(IN_1, IN_2, EN_A, POT_PIN, OPEN_LIMIT, CLOSE_LIMIT, INTERRUPT_PIN, KP, KI, KD);
  uint16_t opening_limit, closing_limit;

  void setup() {
    Serial.begin(115200);                             //Initialize serial communication at 115200 baud rate
    opening_limit = read_16_bit_value_from_eeprom(0); //Read the opening limit from EEPROM
    closing_limit = read_16_bit_value_from_eeprom(2); //Read the closing limit from EEPROM
    valve.init(opening_limit, closing_limit);         //Initialize the valve with the opening and closing limits (Initialize timer interrupt for pid)
  }

  void loop() {
    valve.follow_setpoint(); //PID update
  }

  //Interrupt for pid setpoint update
  ISR(TIMER1_OVF_vect) {
    TCNT1 = ValveNS::TIMER_VALVE_PRELOAD;
    valve.update(&valve);
  }
#endif