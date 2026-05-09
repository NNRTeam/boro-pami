#ifndef I2CCLIENT_H
#define I2CCLIENT_H

#include <Arduino.h>
#include <Wire.h>
#include <Config.h>
#include <Logger/Logger.h>
#include <missionManager/missionManager.h>
#include <Robot/Robot.h>

class SerialClient
{
public:

    SerialClient(Logger& logger, missionManager* missionManager, Robot* robot);

    void run();
    void receiveData();
    void sendData();
    static SerialClient* instanceWrapper;
private:
    void processMessage(String const &receivedData);

    Logger& m_logger;
    missionManager* m_missionManager;
    Robot* m_robot;
    unsigned long long int m_last_odom_send = 0;
    String m_rxBuffer = "";
};

#endif // I2CCLIENT_H