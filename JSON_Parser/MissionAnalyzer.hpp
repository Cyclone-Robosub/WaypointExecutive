#ifndef MISSION_ANALYSER_HPP
#define MISSION_ANALYSER_HPP

#include "../crs_common/position/position.hpp"
#include <nlohmann/json.hpp>
#include <queue>
#include <string>
#include <filesystem>
#include "Task.hpp"

class MissionAnalyzer {
public:
  MissionAnalyzer();
  MissionAnalyzer(const MissionAnalyzer& other); 
  MissionAnalyzer(std::string filePath);
  MissionAnalyzer(std::filesystem::path filePath);
  void parseJSONForMission();
  Task popNextTask();
  bool allTasksComplete();
  MissionAnalyzer& operator=(const MissionAnalyzer& other);
private:
  std::filesystem::path filePath;
  std::queue<Task> mission;
  Position makePositionFromJSON(nlohmann::json::reference jsonData);
  void copyQueue(const std::queue<Task>& other);
};
#endif