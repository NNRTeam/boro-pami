#ifndef MISSIONMANAGER_H
#define MISSIONMANAGER_H

#include <Arduino.h>
#include <missionManager/mission.h>
#include <missionManager/MissionQueue.h>
#include <Logger/Logger.h>

class missionManager
{
public:
    missionManager(Logger& logger);

    bool addMission(Mission const &mission);
    void clearMissions();
    [[nodiscard]] bool removeMission(Mission &mission);
    [[nodiscard]] Mission* getCurrentMission();
    void endCurrentMission();
    Mission* startNextMission();
    [[nodiscard]] Mission* getMissionById(int id);
    [[nodiscard]] int getMissionCount() const;
    [[nodiscard]] bool hasMissions() const;
    [[nodiscard]] bool hasActiveMissions();
    [[nodiscard]] Mission::Type getCurrentMissionType() const;
    [[nodiscard]] bool parseMissionMessage(String const &message, Mission &outMission);
    void cancelAllMissions();
    void startYellowTeamMissions();
    void startBlueTeamMissions();

    void addFakeMissionForTest();
protected:
    MissionQueue missions;

private:
    Logger& m_logger;
};





#endif // MISSIONMANAGER_H