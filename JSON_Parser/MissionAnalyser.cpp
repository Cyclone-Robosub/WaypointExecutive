#include "MissionAnalyser.hpp"
#include <fstream>
#include <iostream>
#include <filesystem>
using json = nlohmann::json;

MissionAnalyser::MissionAnalyser(std::string filePath) : filePath(filePath) {}

Position MissionAnalyser::makePositionFromJSON(json::reference jsonData) {
    auto posArray = jsonData["position"].get<std::array<float, 6>>();
    return Position(posArray);
}

void MissionAnalyser::parseJSONForMission() {
    // Open and read the JSON file
    std::ifstream file(filePath);
    if(!file.is_open()){
        throw std::runtime_error("The Mission path file did not open"); 
    }
    json missionJson;
    file >> missionJson;
    
    // Clear any existing mission tasks
    while (!allTasksComplete()) {
        mission.pop();
    }
    
    for (auto& taskJson : missionJson["tasks"]) { // TODO: Not sure when to set interruptable to true, so it's always false...
        for (auto it = taskJson.begin(); it != taskJson.end(); it++) {
            Task newTask;
            newTask.name = it.key(); // iterator key is name of steps list from JSON (i.e. "Octagon")
            
            for (auto& stepJson : it.value()) { // iterator value is the list elements themselves
                Step step;
                                
                if (stepJson.contains("wait")) {
                    step.WaypointPointer = nullptr; // No position change. TODO: Maybe make waypoint optional?
                    double waitTime = stepJson["wait"].get<double>();
                    step.HoldWaypTime_TimeElapsed = std::make_pair(waitTime, 0.0);
                }
                else if (stepJson.contains("set_control_mode")) { // not sure if this should be handled seperately. What does changing control mode involve?
                    std::string mode = stepJson["set_control_mode"].get<std::string>();
                    if (mode == "vision") {
                        step.VisionCommand = "Dropper";
                    }
                }
                else if (stepJson.contains("waypoint")) {
                    json::reference waypointData = stepJson["waypoint"];

                    step.WaypointPointer = std::make_shared<Position>(makePositionFromJSON(waypointData));
                                        
                    double holdTime = waypointData["hold_time"].get<double>();
                    if (holdTime > 0) {
                        step.HoldWaypTime_TimeElapsed = std::make_pair(holdTime, 0.0);
                    }
                }
                else if (stepJson.contains("barrel_roll")) {
                    json::reference rollData = stepJson["barrel_roll"];

                    step.WaypointPointer = std::make_shared<Position>(makePositionFromJSON(rollData));
                                        
                    step.doBarrelRoll = true;
                    
                    double holdTime = rollData["hold_time"].get<double>();
                    if (holdTime > 0) {
                        step.HoldWaypTime_TimeElapsed = std::make_pair(holdTime, 0.0);
                    }
                }
                else if (stepJson.contains("control_signal")) {
                    std::string signal = stepJson["control_signal"].get<std::string>();
                    if (signal == "STOP") { //Rename "STOP"
                        step.stopWorking = true;
                    }
                    // TODO: add handling for any other signals (not sure if we're going to have any others?)
                }                    
                newTask.steps_queue.push(step);
            }                
            mission.push(newTask);
        }
    }
}

bool MissionAnalyser::allTasksComplete() {
    return mission.empty();
}

Task MissionAnalyser::popNextTask() {
    if (allTasksComplete()) {
        return Task{}; // return empty newTask if queue is empty
    }
    Task nextTask = mission.front();
    mission.pop();
    return nextTask;
}
