<div align="center">  
  <img src="https://www.firstinspires.org/sites/default/files/2024-banner/frc_reefscape.gif" width="300" alt="2025 Reefscape Logo"/><br/>
  <br>
  <h1>Reefscape (2025 Season)</h1>
</div>

## Bot
Please put a finished picture of the bot here when we finish it.

## Goals
- [X] YAGSL
- [X] PathPlanner
- [ ] Tuned values for YAGSL and PathPlanner
- [ ] Elevator subsystem
- [ ] Climber subsystem
- [ ] Coral subsystem
- [X] Algae subsystem

## Development Journey

### Week 3 & 4 (1/20 - 2/1)
* YAGSL now works on unnamed frame bot (UFB). Note to self: make sure to change this when we actually decide on a name for the bot.
  * Demo (for like 2 seconds) then Aaron singing:
  
https://github.com/user-attachments/assets/33cbf16d-e4bf-4471-a0ff-4f62e915d27c

* LED strips are now working (on a separate bot)! It can show all sorts of colors, it can (in theory) show the elevator progress, do segmented rainbow, etc.

### Week 1 & 2 (1/4 - 1/18) - Starting with Remi (Crescendo Bot)
* Begin following a proper command-base project structure.
* YAGSL and PathPlanner have been implemented into Remi!
  * Initially there were some issues with the gyro resetting mid-drive but this was discovered to be the result of a faulty VRM (voltage regulation module).
  * PID constants need to be tuned. PathPlanner especially needs more accurate measurements, there was a lot of error.
* LimeLight implementation has begun! Currently, there is no pose estimation, but the robot can now align itself in front of an AprilTag.
  * It has some jittering issues because the LimeLight's recognition of the AprilTag is sporadic, then the align command doesn't know what to do. This can be resolved by using pose estimation and using the last known pose when the AprilTag is gone.
* Remi has now been handed over to the build team to be scrapped for parts.
