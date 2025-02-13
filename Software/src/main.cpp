#include <constants.h>

#ifdef MAIN_CPP
  #include <valve.h>
  #include <utils.h>

  Valve valve(IN_1, IN_2, EN_A, POT_PIN, OPEN_LIMIT, CLOSE_LIMIT, INTERRUPT_PIN, KP, KI, KD);
  uint16_t opening_limit, closing_limit;

  void setup() {
    Serial.begin(115200);
    opening_limit = read_16_bit_value_from_eeprom(0);
    closing_limit = read_16_bit_value_from_eeprom(2);
    valve.init(opening_limit, closing_limit);
  }

  void loop() {
    valve.follow_setpoint();
  }

  ISR(TIMER1_OVF_vect) {
    TCNT1 = ValveNS::TIMER_VALVE_PRELOAD;
    valve.update(&valve);
  }
#endif