// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class CoralAndProcessorAlign extends Command {
    private PathPlannerTrajectoryState m_currentTrajectory;
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final SwerveSubsystem m_swerveSubsystem;

    public CoralAndProcessorAlign(SwerveSubsystem swerveSubsystem,
            ElevatorSubsystem elevatorSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_swerveSubsystem = swerveSubsystem;

        addRequirements(m_swerveSubsystem);
    }

    @Override
    public void execute() {
        double translationLimit = 1.2 * (1 - (m_elevatorSubsystem.getPosition() / 1.3)) + 0.3;
        double rotationLimit = (5 * Math.PI / 6) * (1 - (m_elevatorSubsystem.getPosition() / 1.3)) + (Math.PI / 6);

        PathPlannerTrajectoryState trajectoryState = new PathPlannerTrajectoryState();
        trajectoryState.pose = m_swerveSubsystem.getPose().nearest(DriveConstants.kInteractionAreas);
        m_currentTrajectory = trajectoryState;

        ChassisSpeeds speeds = DriveConstants.kPathDriveController
                .calculateRobotRelativeSpeeds(m_swerveSubsystem.getPose(), trajectoryState);
        speeds.vxMetersPerSecond = MathUtil.clamp(speeds.vxMetersPerSecond, -translationLimit, translationLimit);
        speeds.vyMetersPerSecond = MathUtil.clamp(speeds.vyMetersPerSecond, -translationLimit, translationLimit);
        speeds.omegaRadiansPerSecond = MathUtil.clamp(speeds.omegaRadiansPerSecond, -rotationLimit, rotationLimit);

        m_swerveSubsystem.getLibSwerveDrive().drive(speeds);
    }

    @Override
    public boolean isFinished() {
        if (m_currentTrajectory == null)
            return false;
        return (m_swerveSubsystem.getPose().getTranslation()
                .getDistance(m_currentTrajectory.pose.getTranslation())) <= 0.02 &&
                PhotonUtils.getYawToPose(m_swerveSubsystem.getPose(), m_currentTrajectory.pose).getDegrees() < 3;
    }
}
