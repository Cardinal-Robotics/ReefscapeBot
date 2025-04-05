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
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class ReefAlign extends Command {
    private final static List<Pose2d> m_blueAprilTags = Arrays.asList(
            VisionSubsystem.getAprilTagPose(17),
            VisionSubsystem.getAprilTagPose(18),
            VisionSubsystem.getAprilTagPose(19),
            VisionSubsystem.getAprilTagPose(20),
            VisionSubsystem.getAprilTagPose(21),
            VisionSubsystem.getAprilTagPose(22));
    private final static List<Pose2d> m_redAprilTags = Arrays.asList(
            VisionSubsystem.getAprilTagPose(6),
            VisionSubsystem.getAprilTagPose(7),
            VisionSubsystem.getAprilTagPose(8),
            VisionSubsystem.getAprilTagPose(9),
            VisionSubsystem.getAprilTagPose(10),
            VisionSubsystem.getAprilTagPose(11));
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    private final SwerveSubsystem m_swerveSubsystem;

    private Transform2d m_poseOffset = setOffsetPose(0, -0.5);
    private PathPlannerTrajectoryState m_currentTrajectory;

    public ReefAlign(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem,
            ElevatorSubsystem elevatorSubsystem, double offsetX,
            double offsetY) {
        this(visionSubsystem, swerveSubsystem, elevatorSubsystem);
        setOffsetPose(offsetX, offsetY);
    }

    public ReefAlign(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem,
            ElevatorSubsystem elevatorSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_visionSubsystem = visionSubsystem;
        m_swerveSubsystem = swerveSubsystem;

        addRequirements(m_visionSubsystem);
        addRequirements(m_swerveSubsystem);
    }

    /**
     * Think of a simple 2D graph where y is height and x is horizontal. To be away
     * from and left of an AprilTag is like quadrant 3 (-,-)
     * 
     * @param x - Positive X = closer to AprilTag; negative X = farther from
     *          AprilTag.
     * @param y - Positive Y = right of AprilTag; negative Y = left of AprilTag.
     * 
     */
    public Transform2d setOffsetPose(double x, double y) {
        return m_poseOffset = new Transform2d(x, y, Rotation2d.kZero);
    }

    @Override
    public void execute() {
        Pose2d pose = m_swerveSubsystem.getPose()
                .nearest(DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue) ? m_blueAprilTags
                        : m_redAprilTags)
                .plus(m_poseOffset);

        // Calculates the propper speed to correctly face the AprilTag (assuming
        // odometry is perfect, because VisionSubsytem::getAprilTagPose doesn't get the
        // actual AprilTag data but it gets where the AprilTag *should* be on the
        // field).

        double translationLimit = 1.2 * (1 - (m_elevatorSubsystem.getPosition() / 1.3)) + 0.3;
        double rotationLimit = (5 * Math.PI / 6) * (1 - (m_elevatorSubsystem.getPosition() / 1.3)) + (Math.PI / 6);

        PathPlannerTrajectoryState trajectoryState = new PathPlannerTrajectoryState();
        trajectoryState.pose = pose.plus(new Transform2d(0, 0, Rotation2d.k180deg));
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
