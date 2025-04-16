#include <constants.h>

#ifdef SETUP_CPP
  #include <valve.h>
  #include <utils.h>

  //Create a valve object
  Valve valve(IN_1, IN_2, EN_A, POT_PIN, OPEN_LIMIT, CLOSE_LIMIT, INTERRUPT_PIN);

  void setup() {
    Serial.begin(115200);                                 //Initialize serial communication at 115200 baud rate
    valve.open_until_hit();                               //Open the valve until the open limit switch is hit
    delay(3000);                                          //Wait for 3 seconds
    save_16_bit_value_to_eeprom(valve.get_position(), 0); //Save the position to EEPROM
    valve.close_until_hit();                              //Close the valve until the close limit switch is hit
    delay(3000);                                          //Wait for 3 seconds
    save_16_bit_value_to_eeprom(valve.get_position(), 2); //Save the position to EEPROM
  }

  void loop() {
    //Print the potenciometer value saved in EEPROM for open and close positions
    Serial.print(read_16_bit_value_from_eeprom(0));
    Serial.print(" ");
    Serial.println(read_16_bit_value_from_eeprom(2));
  }
#endif