#include "missionManager/missionManager.h"
#include <Utils.h>

missionManager::missionManager(Logger& logger): m_logger(logger){
}

bool missionManager::addMission(Mission const &mission)
{
    m_logger.info("Adding mission ID: " + String(mission.getId()) +
                " Type: " + Mission::typeToString(mission.getType()));
    if (mission.getType() == Mission::Type::STOP)
    {
        m_logger.info("STOP mission received. Clearing all missions.");
        clearMissions();
        return true;
    }
    else if (mission.getType() == Mission::Type::RESUME)
    {
        Mission* current = getCurrentMission();
        if (current != nullptr && current->getType() == Mission::Type::WAIT)
        {
            endCurrentMission();
            startNextMission();
        }
        else
        {
            m_logger.warn("RESUME mission received but current mission is not WAIT (or no current mission).");
        }
        return true;
    }
    if (!missions.push(mission)) {
        m_logger.error("Mission queue full, cannot add mission ID: " + String(mission.getId()));
        return false;
    }
    return true;
}

void missionManager::clearMissions()
{
    missions.clear();
}

[[nodiscard]] bool missionManager::removeMission(Mission &mission)
{
    // Chercher si la mission existe dans la queue
    bool found = false;
    for (int i = 0; i < missions.size(); ++i) {
        if (*missions.at(i) == mission) {
            found = true;
            break;
        }
    }
    if (found)
    {
        cancelAllMissions();
        return true;
    }
    return false;
}

[[nodiscard]] Mission* missionManager::getCurrentMission()
{
    return missions.front();
}

void missionManager::endCurrentMission()
{
    if (hasActiveMissions())
    {
        Mission* currentMission = getCurrentMission();
        currentMission->setStatus(Mission::Status::FINISHED);
        m_logger.info("Ending mission ID: " + String(currentMission->getId()));
        missions.pop();
    }
}

Mission* missionManager::startNextMission()
{
    if (missions.empty())
        return nullptr;

    Mission* currentMission = getCurrentMission();
    currentMission->setStatus(Mission::Status::STARTED);
    m_logger.info("Starting mission ID: " + String(currentMission->getId()));
    return currentMission;
}

[[nodiscard]] Mission* missionManager::getMissionById(int id)
{
    for (int i = 0; i < missions.size(); ++i)
    {
        Mission* m = missions.at(i);
        if (m->getId() == id)
            return m;
    }
    return nullptr;
}

[[nodiscard]] int missionManager::getMissionCount() const
{
    return missions.size();
}

[[nodiscard]] bool missionManager::hasMissions() const
{
    return !missions.empty();
}

[[nodiscard]] bool missionManager::hasActiveMissions()
{
    return !missions.empty() && missions.front()->isActive();
}

[[nodiscard]] Mission::Type missionManager::getCurrentMissionType() const
{
    if (missions.empty())
        return Mission::Type::NONE;

    return missions.front()->getType();
}

[[nodiscard]] bool missionManager::parseMissionMessage(String const &message, Mission &outMission)
{
    // message format: "{ID};{TYPE_INT};{STATUS_INT};{OPTION_INT};{DIRECTION_INT};{TARGET_X_FLOAT};{TARGET_Y_FLOAT};{TARGET_THETA_FLOAT}"
    // Parse sans allocation dynamique : on cherche les positions des ';'
    int separators[7];
    int sepCount = 0;
    for (size_t i = 0; i < message.length() && sepCount < 7; ++i)
    {
        if (message.charAt(i) == ';')
            separators[sepCount++] = i;
    }
    if (sepCount != 7)
    {
        m_logger.error("Invalid mission message format: " + message);
        return false;
    }

    int id = message.substring(0, separators[0]).toInt();
    int typeInt = message.substring(separators[0] + 1, separators[1]).toInt();
    int statusInt = message.substring(separators[1] + 1, separators[2]).toInt();
    int optionsInt = message.substring(separators[2] + 1, separators[3]).toInt();
    int directionInt = message.substring(separators[3] + 1, separators[4]).toInt();
    float target_x = message.substring(separators[4] + 1, separators[5]).toFloat();
    float target_y = message.substring(separators[5] + 1, separators[6]).toFloat();
    float target_theta = message.substring(separators[6] + 1).toFloat();

    // Validation des valeurs d'enum
    if (typeInt < 0 || typeInt > 5) {
        m_logger.error("Invalid mission type: " + String(typeInt));
        return false;
    }
    if (statusInt < 0 || statusInt > 4) {
        m_logger.error("Invalid mission status: " + String(statusInt));
        return false;
    }

    outMission = Mission(
        id,
        static_cast<Mission::Type>(typeInt),
        static_cast<Mission::Options>(optionsInt),
        static_cast<Mission::Direction>(directionInt),
        target_x, target_y, target_theta
    );
    outMission.setStatus(static_cast<Mission::Status>(statusInt));
    return true;
}

void missionManager::cancelAllMissions()
{
    while (!missions.empty()) {
        missions.front()->setStatus(Mission::Status::CANCELED);
        missions.pop();
    }
}

void missionManager::startYellowTeamMissions()
{
    m_logger.info("Starting yellow team missions");
    Mission mission1(1, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.5f, 0.0f, 0.0f);
    addMission(mission1);
    Mission mission2(2, Mission::Type::TURN, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, 3.14159f/2.0f);
    addMission(mission2);
    Mission mission3(3, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.5f, 0.5f, 0.0f);
    addMission(mission3);
}

void missionManager::startBlueTeamMissions()
{
    m_logger.info("Starting blue team missions");
    Mission mission1(1, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.5f, 0.0f, 0.0f);
    addMission(mission1);
    Mission mission2(2, Mission::Type::TURN, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, -3.14159f/2.0f);
    addMission(mission2);
    Mission mission3(3, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.5f, -0.5f, 0.0f);
    addMission(mission3);
}

void missionManager::addFakeMissionForTest()
{
    Mission mission1(1, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.5f, 0.0f, 0.0f);
    addMission(mission1);
    Mission mission2(2, Mission::Type::TURN, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, 3.14159f/2.0f);
    addMission(mission2);
    Mission mission3(3, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.5f, 0.5f, 0.0f);
    addMission(mission3);
    Mission mission4(4, Mission::Type::TURN, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, 3.14159f);
    addMission(mission4);
    Mission mission5(5, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.5f, 0.0f);
    addMission(mission5);
    Mission mission6(6, Mission::Type::TURN, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, -3.14159f/2.0f);
    addMission(mission6);
    Mission mission7(7, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, 0.0f);
    addMission(mission7);
    Mission mission8(8, Mission::Type::TURN, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, 0.0f);
    addMission(mission8);
    Mission mission11(11, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.5f, 0.0f, 0.0f);
    addMission(mission11);
    Mission mission12(12, Mission::Type::TURN, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, 3.14159f/2.0f);
    addMission(mission12);
    Mission mission13(13, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.5f, 0.5f, 0.0f);
    addMission(mission13);
    Mission mission14(14, Mission::Type::TURN, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, 3.14159f);
    addMission(mission14);
    Mission mission15(15, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.5f, 0.0f);
    addMission(mission15);
    Mission mission16(16, Mission::Type::TURN, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, -3.14159f/2.0f);
    addMission(mission16);
    Mission mission17(17, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, 0.0f);
    addMission(mission17);
    Mission mission18(18, Mission::Type::TURN, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, 0.0f);
    addMission(mission18);
}
