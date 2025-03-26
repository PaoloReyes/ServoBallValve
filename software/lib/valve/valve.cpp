#include "valve.h"

// Static variables initialization
uint32_t Valve::rising_timestamp = 0;
uint32_t Valve::falling_timestamp = 0;
uint32_t Valve::last_falling_timestamp = 0;
uint32_t Valve::up_time = 0;
uint32_t Valve::down_time = 0;
double Valve::setpoint_angle = 0.0;
uint8_t Valve::interrupt_pin = 2;

/// @brief Valve pins and interrupt pulse analysis initialization
/// @param in_1 Input 1 pin
/// @param in_2 Input 2 pin
/// @param ena Enable pin
/// @param pot_pin Potentiometer pin
/// @param open_limit Limit switch for open position
/// @param close_limit Close switch for close position
/// @param interrupt_pin Interrupt pin for pulse analysis
Valve::Valve(uint8_t in_1, uint8_t in_2, uint8_t ena, uint8_t pot_pin, uint8_t open_limit, uint8_t close_limit, uint8_t interrupt_pin) {
    this->in_1 = in_1;
    this->in_2 = in_2;
    this->ena = ena;
    this->pot_pin = pot_pin;
    this->open_limit = open_limit;
    this->close_limit = close_limit;
    Valve::interrupt_pin = interrupt_pin;
    pinMode(this->pot_pin, INPUT);
    pinMode(Valve::interrupt_pin, INPUT);
    pinMode(this->open_limit, INPUT_PULLUP);
    pinMode(this->close_limit, INPUT_PULLUP);
    pinMode(this->in_1, OUTPUT);
    pinMode(this->in_2, OUTPUT);
    pinMode(this->ena, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(Valve::interrupt_pin), pulse_analizer, CHANGE);
}

/// @brief Valve pins and interrupt pulse analysis initialization with PID constants
/// @param in_1 Input 1 pin
/// @param in_2 Input 2 pin
/// @param ena Enable pin
/// @param pot_pin Potentiometer pins

/// @param open_limit Limit switch for open position
/// @param close_limit Close switch for close position
/// @param interrupt_pin Interrupt pin for pulse analysis
/// @param kp Proportional constant
/// @param ki Integral constant
/// @param kd Derivative constant
Valve::Valve(uint8_t in_1, uint8_t in_2, uint8_t ena, uint8_t pot_pin, uint8_t open_limit, uint8_t close_limit, uint8_t interrupt_pin, double kp, double ki, double kd) : Valve(in_1, in_2, ena, pot_pin, open_limit, close_limit, interrupt_pin) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

/// @brief Setpoint extraction from servo-like pulse analysis
void Valve::pulse_analizer() {
    // First edge discovery
    if (Valve::last_falling_timestamp == 0 && digitalRead(Valve::interrupt_pin) == 0) {
        Valve::last_falling_timestamp = micros();
    } else {
        // Edge detection
        if (digitalRead(Valve::interrupt_pin)) {
            Valve::rising_timestamp = micros();
        } else {
            Valve::last_falling_timestamp = Valve::falling_timestamp;
            Valve::falling_timestamp = micros();
        }
        // Valve pulse analysis
        if (Valve::falling_timestamp-Valve::rising_timestamp < 3000) {
            // Valve pulse width calculation
            Valve::up_time = Valve::falling_timestamp-Valve::rising_timestamp;
            Valve::down_time = Valve::rising_timestamp-Valve::last_falling_timestamp;
            // Frequency validation
            if (abs(20000-(int32_t)(Valve::up_time+Valve::down_time)) < 100) {
                Valve::setpoint_angle = constrain_double(map_double(Valve::up_time, 1000.0, 2000.0, 0.0, 90.0), 0, 90);
            }
        }
    }
}

/// @brief Timer1 interrupt service routine for PID control and limit values initialization
/// @param open_limit_value Valve open limit value (Potentiometer value)
/// @param close_limit_value Valve close limit value (Potentiometer value)
void Valve::init(uint16_t open_limit_value, uint16_t close_limit_value) {
    TCCR1A = 0;              // Initialize Timer1 Control Registers
    TCCR1B = 0;              // Initialize Timer1 Control Registers
    TCCR1B |= (ValveNS::TIMER_VALVE_PRESCALAR == 64)? 0b00000011: (ValveNS::TIMER_VALVE_PRESCALAR == 8)? 0b00000010 : 0b00000001; // Set Prescalar
    TCNT1 = ValveNS::TIMER_VALVE_PRELOAD; // Timer Preloading
    TIMSK1 |= B00000001;     // Enable Timer Overflow Interrupt

    this->open_limit_value = open_limit_value;
    this->close_limit_value = close_limit_value;
}

/// @brief Setpoint angle setter
/// @param setpoint_angle Angle to be set
void Valve::set_angle(uint16_t setpoint_angle) {
    Valve::setpoint_angle = setpoint_angle;
}

/// @brief Open valve until hit the open limit switch at 60% speed
void Valve::open_until_hit() {
    while (this->is_open()) {
        this->open(100);
        delay(10);
    }
    this->stop();
}

/// @brief Close valve until hit the close limit switch at 60% speed
void Valve::close_until_hit() {
    while (this->is_closed()) {
        this->close(100);
        delay(10);
    }
    this->stop();
}

/// @brief Check if valve is open
/// @return 1 if open, 0 if closed
bool Valve::is_open() {
    return digitalRead(this->open_limit);
}

/// @brief Check if valve is closed
/// @return 1 if closed, 0 if open
bool Valve::is_closed() {
    return digitalRead(this->close_limit);
}

/// @brief Take action to follow setpoint angle based on PID output on Timer1 interrupt
void Valve::follow_setpoint() {
    this->move(this->pid_output);
    Serial.print("Setpoint:");
    Serial.print(Valve::setpoint_angle);
    Serial.print(",");
    Serial.print("Position:");
    Serial.println(this->analog2degrees(this->get_position()));
}

/// @brief Map analog value from potentiometer to degrees based on valve limits
/// @param analog Raw analog value from potentiometer
/// @return 0-90 degrees mapped value
double Valve::analog2degrees(uint16_t analog) {
    return map_double(analog, this->close_limit_value, this->open_limit_value, 0.0, 90.0);
}

/// @brief Get potentiometer value
/// @return Potentiometer value
uint16_t Valve::get_position() {
    return analogRead(this->pot_pin);
}

/// @brief PID calculation for valve control on Timer1 interrupt
/// @param valve Valve object reference to be updated
void Valve::update(Valve* valve) {
    double error = Valve::setpoint_angle - valve->analog2degrees(valve->get_position());

    if (abs(error) < 0.5) {
        valve->pid_output = 0.0;
        valve->last_error = error;
        return;
    }

    valve->integral_error += error * ValveNS::TIMER_VALVE_S;
    if (abs(error) > 4.0 || valve->last_error * error < 0) {
        valve->integral_error = 0.0;
    }
    valve->derivative_error = (error - valve->last_error) / ValveNS::TIMER_VALVE_S;
    valve->pid_output = valve->kp * error + valve->ki * valve->integral_error + valve->kd * valve->derivative_error;
    valve->last_error = error;
}

/// @brief Move valve on [-100, 100] speed range
/// @param speed Speed value
void Valve::move(int16_t speed) {
    speed = constrain(speed, -100, 100);
    if ((!this->is_open() && Valve::setpoint_angle > 88.5) || (!this->is_closed() && Valve::setpoint_angle < 1.51) || speed == 0) {
        this->stop();
    } else {
        speed>0?this->open(speed):this->close(-speed);
    }
}

/// @brief Open valve at speed [0, 100]
/// @param speed Speed value
void Valve::open(uint8_t speed) {
    speed = map(speed, 0, 100, 0, 255);
    analogWrite(this->ena, speed);
    digitalWrite(this->in_1, HIGH);
    digitalWrite(this->in_2, LOW);
}

/// @brief Close valve at speed [0, 100]
/// @param speed Speed value
void Valve::close(uint8_t speed) {
    speed = map(speed, 0, 100, 0, 255);
    analogWrite(this->ena, speed);
    digitalWrite(this->in_1, LOW);
    digitalWrite(this->in_2, HIGH);
}

/// @brief Stop valve
void Valve::stop() {
    analogWrite(this->ena, 0);
    digitalWrite(this->in_1, LOW);
    digitalWrite(this->in_2, LOW);
}