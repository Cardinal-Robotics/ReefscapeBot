// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.GenericMotor;
import frc.robot.visual.GenericMotorVisual;

public class RobotContainer {
    /* Lessons: */
    // private final GenericMotor m_genericMotor = new GenericMotor();

    /* Challenge: */
    // private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();

    public RobotContainer() {
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
