#include "SerialClient/SerialClient.h"

SerialClient* SerialClient::instanceWrapper = nullptr; // or appropriate initial value

SerialClient::SerialClient(Logger& logger, missionManager* missionManager, Robot* robot) :
    m_logger(logger), m_missionManager(missionManager), m_robot(robot) {}

void SerialClient::run()
{
    receiveData();
    unsigned long long int const current_time = micros();
    if (current_time - m_last_odom_send > 1e6/config::ODOM_FREQUENCY_HZ)
    {
        m_last_odom_send = current_time;
        sendData();
    }

}

void SerialClient::receiveData() {
    if (!Serial.available())
        return;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == 'F') {
            processMessage(m_rxBuffer);
            m_rxBuffer = "";
            continue;
        }

        if (c == '\r' || c == '\n') {
            continue;
        }

        m_rxBuffer += c;

        // Guard against malformed streams with missing terminator.
        if (m_rxBuffer.length() > 160) {
            m_logger.warn("RX buffer overflow, dropping partial frame: " + m_rxBuffer);
            m_rxBuffer = "";
        }
    }
}

void SerialClient::processMessage(String const &receivedData)
{
    if (receivedData.length() == 0) {
        return;
    }

    if (receivedData[0] == 'R')
    {
        m_robot->resetOdometry();
    }
    else if (receivedData[0] == 'L')
    {
        // emergency stop command, set max deceleration to a high value to stop the robot as fast as possible
        m_robot->emergencyStop(true);
    }
    else if (receivedData[0] == 'O')
    {
        m_robot->parseOdometryData(receivedData);
    }
    else
    {
        Mission mission;
        if (m_missionManager->parseMissionMessage(receivedData, mission)) {
            m_missionManager->addMission(mission);
            m_robot->emergencyStop(false);
        }
    }
}

void writeUint32(uint32_t value) {
    Serial.write((uint8_t)(value >> 24)); // Octet le plus significatif
    Serial.write((uint8_t)(value >> 16));
    Serial.write((uint8_t)(value >> 8));
    Serial.write((uint8_t)value);         // Octet le moins significatif
}

void SerialClient::sendData() {
    //Mission* currentMission = m_missionManager->getCurrentMission();
    float const x = m_robot->getX();
    float const y = m_robot->getY();
    float const theta = m_robot->getTheta();
    float const vl = m_robot->getLinearSpeed();
    float const vr = m_robot->getAngularSpeed();
    String d = "O" + String(x) + ";" + String(y) + ";" + String(theta) + ";" + String(vl) + ";" + String(vr) + "F";
    Serial.println(d);
}