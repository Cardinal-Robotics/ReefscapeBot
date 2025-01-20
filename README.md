<div align="center">  
  <img height="300" src="https://info.firstinspires.org/hs-fs/hubfs/2025%20Season/Season%20Assets/FIRST_DIVE-reefscape-PatchLogo.png?width=283&height=343&name=FIRST_DIVE-reefscape-PatchLogo.png"  />
  <br>
  <h1>ReefscapeBot (2025 Season)</h1>
</div>

## Goals
- [X] YAGSL
- [X] PathPlanner
- [ ] Tuned values for YAGSL and PathPlanner
- [ ] Elevator subsystem
- [ ] Climber subsystem

## Development Journey

### Week 3 & 4 (1/20 - 2/1)

### Week 1 & 2 (1/4 - 1/18) - Starting with Remi (Crescendo Bot)
* Begin following a proper command-base project structure.
* YAGSL and PathPlanner have been implemented into Remi!
  * Initially there were some issues with the gyro resetting mid-drive but this was discovered to be the result of a faulty VRM (voltage regulation module).
  * PID constants need to be tuned. PathPlanner especially needs more accurate measurements, there was a lot of error.
* LimeLight implementation has begun! Currently, there is no pose estimation, but the robot can now align itself in front of an AprilTag.
  * It has some jittering issues because the LimeLight's recognition of the AprilTag is sporadic, then the align command doesn't know what to do. This can be resolved by using pose estimation and using the last known pose when the AprilTag is gone.
* Remi has now been handed over to the build team to be scrapped for parts.