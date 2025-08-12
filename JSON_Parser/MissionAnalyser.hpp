#ifndef MISSION_ANALYSER_HPP
#define MISSION_ANALYSER_HPP


#include "../../crs_common/position/position.hpp"
#include "../lib/JSON/Json.hpp"
#include "../src/Task.hpp"
#include <queue>
#include <string>
#include <filesystem>

struct Task;
class MissionAnalyser {
public:
  MissionAnalyser();
  MissionAnalyser(std::string filePath);
  MissionAnalyser(std::filesystem::path filePath);
  void parseJSONForMission();
  Task popNextTask();
  bool allTasksComplete();

private:
  std::string filePath;
  std::queue<Task> mission;
  Position makePositionFromJSON(nlohmann::json::reference jsonData);
};
#endif