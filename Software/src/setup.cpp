#include <constants.h>

#ifdef SETUP_CPP
  #include <valve.h>
  #include <utils.h>

  Valve valve(IN_1, IN_2, EN_A, POT_PIN, OPEN_LIMIT, CLOSE_LIMIT, INTERRUPT_PIN);

  void setup() {
    Serial.begin(115200);
    valve.open_until_hit();
    delay(3000);
    save_16_bit_value_to_eeprom(valve.get_position(), 0);
    valve.close_until_hit();
    delay(3000);
    save_16_bit_value_to_eeprom(valve.get_position(), 2);
  }

  void loop() {
    Serial.print(read_16_bit_value_from_eeprom(0));
    Serial.print(" ");
    Serial.println(read_16_bit_value_from_eeprom(2));
  }
#endif