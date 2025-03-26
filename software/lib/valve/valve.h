#ifndef VALVE_H
    #define VALVE_H

    #include <Arduino.h>
    #include <utils.h>

    namespace ValveNS {
        const uint16_t TIMER_VALVE_MAX = 65535;   // 16-bit timer max value
        const double TIMER_VALVE_S = 0.1;         // Timer1 interrupt period in seconds
        const uint8_t TIMER_VALVE_PRESCALAR = 64; // Timer1 prescalar
        const uint16_t TIMER_VALVE_PRELOAD = ValveNS::TIMER_VALVE_MAX-(ValveNS::TIMER_VALVE_S*F_CPU/ValveNS::TIMER_VALVE_PRESCALAR); // Timer1 preload value
    }

    class Valve {
        public:
            static uint32_t rising_timestamp, falling_timestamp, last_falling_timestamp, up_time, down_time;
            static uint8_t interrupt_pin;
            static double setpoint_angle;
            static void pulse_analizer(void);

            Valve(uint8_t in_1, uint8_t in_2, uint8_t ena, uint8_t pot_pin, uint8_t open_limit, uint8_t close_limit, uint8_t interrupt_pin);
            Valve(uint8_t in_1, uint8_t in_2, uint8_t ena, uint8_t pot_pin, uint8_t open_limit, uint8_t close_limit, uint8_t interrupt_pin, double kp, double ki, double kd);

            void init(uint16_t opening_limit, uint16_t closing_limit);

            void set_angle(uint16_t angle);

            void open_until_hit(void);
            void close_until_hit(void);

            bool is_open(void);
            bool is_closed(void);

            void follow_setpoint();

            double analog2degrees(uint16_t analog);
            uint16_t get_position();

            void update(Valve *valve);

        private:
            uint8_t in_1, in_2, ena, pot_pin, open_limit, close_limit;
            uint16_t open_limit_value, close_limit_value;
            double kp, ki, kd, integral_error, derivative_error, last_error, pid_output;

            void move(int16_t speed);
            void open(uint8_t speed);
            void close(uint8_t speed);
            void stop();
    };
#endif