#ifndef motor_control_hpp
#define motor_control_hpp

#include "HallSensor/HallSensor.h"
#include "Arduino.h"


class Motor{
    public:
        Motor(int dirPin, int stepPin, int sensorCS, bool invertSensor, bool invertMotor, double wheelPerimeter, void (*stepCallback)(bool forward));
        void run();
        void runFromTimerISR();
        void setLinearSpeed(float speed);
        void stop();
        void UpdateWeelPerimeter(double weelPerimeter) { m_WheelPerimeter = weelPerimeter; }
        [[nodiscard]] float getWheelPerimeter() const { return m_WheelPerimeter; }
        [[nodiscard]] float getFeedbackSpeed(unsigned int *dt, unsigned int *t);
    protected:

        void setSpeed(float speed);
        void step();

        void (*m_stepCallback)(bool forward);
        int PIN_DIR;          //Pin de direction
        int PIN_STEP;         //Pin de step

        float m_speed;         //Consigne de vitesse angulaire [rd.sec-1]

        volatile unsigned int m_interStepTime = 0; // Cached inter-step time in µs (0 = stopped)
        volatile bool m_cachedWheelForward = true;  // Cached wheel direction for ISR
        volatile unsigned int m_ticksPerStep = 0;
        volatile unsigned int m_ticksUntilStep = 0;
        volatile bool m_stepPulseHigh = false;

        uint64_t m_dt=0.0;                //delta t
        uint64_t m_t=0.0;                 //temps

        double m_WheelPerimeter;     //perimetre de la roue [mm]

        HallSensor* m_sensor = nullptr;
        bool m_is_sensor_inverted;
        bool m_is_motor_inverted;

        unsigned long long int m_lastStepTime = 0;
};





#endif