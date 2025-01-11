// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveDrive m_swerveDrive;

    public SwerveSubsystem() {
        try {
            m_swerveDrive = new SwerveParser(DriveConstants.kSwerveDirectory)
                    .createSwerveDrive(DriveConstants.kMaxSpeed);
        } catch (Exception e) {
            SmartDashboard.putString("SwerveSubsystem", "Failed to create YAGSL Swerve Drive");
            throw new RuntimeException(e);
        }
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        m_swerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            this.m_swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    public SwerveDrive getSwerveDrive() {
        return this.m_swerveDrive;
    }
}
