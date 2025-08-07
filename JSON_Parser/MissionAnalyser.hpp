#ifndef MISSION_ANALYSER_HPP
#define MISSION_ANALYSER_HPP

#include "../../../crs_common/position/position.hpp"
#include "../../lib/Json/json.hpp"
#include "../Task.hpp"
#include <queue>
#include <string>

struct Task;
class MissionAnalyser {
public:
  MissionAnalyser(std::string filePath);
  void parseJSONForMission();
  Task popNextTask();
  bool allTasksComplete();

private:
  std::string filePath;
  std::queue<Task> mission;
  Position makePositionFromJSON(nlohmann::json::reference jsonData);
};
#endif