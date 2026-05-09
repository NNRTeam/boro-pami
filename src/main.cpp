#include <Arduino.h>
#include <Config.h>
#include <Logger/Logger.h>
#include <SystemManager/SystemManager.h>

SystemManager systemManager;

#define START_PIN 14
#define TEAM_PIN 15

bool isYellowTeam = false;

void setup() {
    Serial.begin(config::SERIAL_BAUDRATE);
    SPI.begin();
    systemManager.initialize(Logger::Level::INFO);
    //systemManager.m_missionManager->addFakeMissionForTest();

    pinMode(START_PIN, INPUT);
    pinMode(TEAM_PIN, INPUT);
}

bool match_started = false;

void loop() {
    systemManager.m_robot->run();
    systemManager.m_serialClient->run();

    if (!match_started)
    {
        isYellowTeam = digitalRead(TEAM_PIN) == HIGH;
        if (digitalRead(START_PIN) == LOW) {
            match_started = true;
            Serial.println("Match started! Team: " + String(isYellowTeam ? "Yellow" : "Blue"));
            delay((100-15)*1000);
            if (isYellowTeam) {
                systemManager.m_missionManager->startYellowTeamMissions();
            } else {
                systemManager.m_missionManager->startBlueTeamMissions();
            }
    
        }
    }
}