task[len] - one of waypoint/wait: must be at least one 
    - task:
        1. set_control_mode: one of "vison" or "waypoint" or "manipulation": mandatory
            1. waypoint: mandatory
                1.  position: array floats[6] (x,y,z,roll, pitch, yaw)
        2. 
        3. hold_time: optional double: 
        4. max_time: optional int: 
    - wait: double

If (vision or manipulation) and position is not given:
    the hold_time of the step will be mandatory
Otherwise:
    the hold_time is optional