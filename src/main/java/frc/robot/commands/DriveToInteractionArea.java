// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.DriveConstants;

public class DriveToInteractionArea extends Command {
    private final SwerveSubsystem m_swerve;
    private Command m_driveCommand;

    public DriveToInteractionArea(SwerveSubsystem swerveDrive) {
        m_swerve = swerveDrive;
    }

    @Override
    public void initialize() {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_driveCommand == null || m_driveCommand.isFinished();
    }
}
