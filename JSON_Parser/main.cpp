#include <iostream>
#include <string>
#include "MissionAnalyser.hpp"

int main() {
    auto analyser = MissionAnalyser("MissionPath.JSON");
    analyser.parseJSONForMission();

    
    for (int taskCount = 0; !analyser.allTasksComplete(); taskCount++) {
        Task task = analyser.popNextTask();
        
        std::cout << "\nTask " << taskCount << ": " << task.name << std::endl;
        
        int stepCount = 0;
        std::queue<Step> steps = task.steps;
        
        while (!steps.empty()) {
            Step step = steps.front();
            steps.pop();
            stepCount++;
            
            std::cout << "  Step " << stepCount << ":" << std::endl;
            
            if (step.WaypointPointer) {
                std::cout << "      Is waypoint: Yes" << std::endl;
            } else {
                std::cout << "      Is waypoint: No" << std::endl;
            }
            
            std::cout << "      Needs vision: " << (step.NeedsVision ? "Yes" : "No") << std::endl;
            std::cout << "      Is interruptable: " << (step.isInterruptable ? "Yes" : "No") << std::endl;
            std::cout << "      Do barrel roll: " << (step.doBarrelRoll ? "Yes" : "No") << std::endl;
            
            if (step.HoldWaypTime_TimeElapsed.has_value()) {
                std::cout << "      Hold time: " << step.HoldWaypTime_TimeElapsed->first << " seconds" << std::endl;
            }

            if (step.stopWorking) {
                std::cout << "      Control signal: STOP" << std::endl;
            }
        }
    }
    
    std::cout << "Mission Parser Demo Complete Successfully" << std::endl;
    
    return 0;
}
