#include "MissionAnalyser.hpp"
#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp> // assuming ../lib/JSON/Json.hpp brings this in

MissionAnalyser::MissionAnalyser() = default;

MissionAnalyser::MissionAnalyser(const MissionAnalyser& other) {
    filePath = other.filePath;
    copyQueue(other.mission);
}

MissionAnalyser::MissionAnalyser(std::string filePath)
    : filePath(std::move(filePath)) {}

MissionAnalyser::MissionAnalyser(std::filesystem::path filePath)
    : filePath(std::move(filePath)) {}

MissionAnalyser& MissionAnalyser::operator=(const MissionAnalyser& other) {
    if (this != &other) {
        filePath = other.filePath;
        copyQueue(other.mission);
    }
    return *this;
}

void MissionAnalyser::copyQueue(const std::queue<Task>& other) {
    std::queue<Task> tmp = other;
    mission = {};
    while (!tmp.empty()) {
        mission.push(tmp.front());
        tmp.pop();
    }
}

Position MissionAnalyser::makePositionFromJSON(nlohmann::json::reference jsonData) {
    if (!jsonData.is_array() || jsonData.size() != 6) {
        throw std::runtime_error("Position must be an array of exactly 6 numbers.");
    }
    return Position(
        jsonData[0].get<double>(),
        jsonData[1].get<double>(),
        jsonData[2].get<double>(),
        jsonData[3].get<double>(),
        jsonData[4].get<double>(),
        jsonData[5].get<double>()
    );
}

void MissionAnalyser::parseJSONForMission() {
    std::ifstream inFile(filePath);
    if (!inFile.is_open()) {
        throw std::runtime_error("Could not open mission file: " + filePath.string());
    }

    nlohmann::json jsonData;
    inFile >> jsonData;

    if (!jsonData.is_array()) {
        throw std::runtime_error("Top-level JSON must be an array of tasks.");
    }

    for (auto& taskJson : jsonData) {
        Task task;
        task.name = taskJson.at("name").get<std::string>();

        if (!taskJson.contains("steps") || !taskJson["steps"].is_array()) {
            throw std::runtime_error("Task \"" + task.name + "\" must contain a 'steps' array.");
        }

        for (auto& stepJson : taskJson["steps"]) {
            Step step;

            std::string type = stepJson.at("type").get<std::string>();

            // Waypoint / Vision / Manipulation may have position
            if (stepJson.contains("position")) {
                Position pos = makePositionFromJSON(stepJson["position"]);
                step.WaypointPointer = std::make_shared<Position>(pos);
            }

            // Hold_time
            if (stepJson.contains("hold_time")) {
                double holdTime = stepJson["hold_time"].get<double>();
                step.HoldWaypTime_TimeElapsed = std::make_pair(holdTime, 0.0);
            }

            // MaxTime
            if (stepJson.contains("maxtime")) {
                step.MaxTime = stepJson["maxtime"].get<unsigned int>();
            }

            // Vision command
            if (type == "vision" && stepJson.contains("vision_command")) {
                std::string cmd = stepJson["vision_command"].get<std::string>();
                step.VisionINTCommand_Serviced = std::make_pair(cmd, false);
            }

            // Manipulation command
            if (type == "manipulation" && stepJson.contains("manipulation_command")) {
                int cmd = stepJson["manipulation_command"].get<int>();
                step.ManipulationCodeandStatus = std::make_pair(cmd, false);
            }

            task.steps_queue.push(step);
        }

        mission.push(task);
    }
}

Task MissionAnalyser::popNextTask() {
    if (mission.empty()) {
        throw std::runtime_error("No more tasks in mission queue.");
    }
    Task next = mission.front();
    mission.pop();
    return next;
}

bool MissionAnalyser::allTasksComplete() {
    return mission.empty();
}
