#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Config.h>
#include <Logger/Logger.h>
#include <missionManager/missionManager.h>
#include <Robot/Motor.h>

class Robot
{
    public:
        Robot(Logger& logger, missionManager* missionManager);

        void run();
        void resetOdometry();
        void parseOdometryData(String const &message);
        void emergencyStop(bool enable);
        [[nodiscard]] float getX() const { if (config::MOTOR_ODOM_ONLY) return m_motorX; else return x; }
        [[nodiscard]] float getY() const { if (config::MOTOR_ODOM_ONLY) return m_motorY; else return y; }
        [[nodiscard]] float getTheta() const { if (config::MOTOR_ODOM_ONLY) return m_motorTheta; else return theta; }
        [[nodiscard]] float getLinearSpeed() const { if (config::MOTOR_ODOM_ONLY) return m_linearSpeedMotor; else return m_linearSpeed; }
        [[nodiscard]] float getAngularSpeed() const { if (config::MOTOR_ODOM_ONLY) return m_angularSpeedMotor; else return m_angularSpeed; }
        static void leftMotorStepNotify(bool forward);
        static void rightMotorStepNotify(bool forward);
        static void stepTimerISR();
        static Robot* instance;

    protected:
        void Control();
        void updateOdometry();
        void updateMotorOdometry();
        void setSpeeds(float linearSpeed, float angularSpeed);
        void samsonUpdateMotors(Mission* mission);
        void rotationUpdateMotors();
        void stop();
        bool checkMissionArrived();
        void setEmergencyStopMotorSpeed();

        Logger& m_logger;

        float x = 0.0; //m
        float y = 0.0; //m
        float theta = 0.0; //rad

        float m_motorX = 0.0; //m
        float m_motorY = 0.0; //m
        float m_motorTheta = 0.0; //rad

        missionManager* m_missionManager = nullptr;

        Motor* motor_left;
        Motor* motor_right;

        float m_linearSpeed = 0.0; //m/s
        float m_angularSpeed = 0.0; //rad/s

        float m_lastLinearSpeed = 0.0; //m/s
        float m_lastAngularSpeed = 0.0; //rad/s

        float m_linearSpeedMotor = 0.0; //m/s
        float m_angularSpeedMotor = 0.0; //rad/s

        float m_currentLinearAccel = 0.0; //m/s^2 - accélération actuelle pour limiter le jerk

        unsigned long long int m_lastControlTime = 0;
        unsigned long long int m_lastOdomTime = 0;
        unsigned long long int m_lastAssignedMission = 0;

        bool m_isEmergencyStop = false;

        volatile long m_leftStepCount = 0;
        volatile long m_rightStepCount = 0;
    private:
        unsigned long m_dt1 = 0;
        unsigned long m_dt2 = 0;
        unsigned long m_dt = (unsigned long)(1000000.0f / config::CONTROL_LOOP_FREQUENCY_HZ);
        unsigned long m_t = 0;

};


#endif // ROBOT_H