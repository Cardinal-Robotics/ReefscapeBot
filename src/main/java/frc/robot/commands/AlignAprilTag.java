// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Cameras;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Optional;

public class AlignAprilTag extends Command {
    private final VisionSubsystem m_visionSubsystem;
    private final SwerveSubsystem m_swerveSubsystem;

    private Transform2d m_poseOffset = setOffsetPose(0, -0.5);
    private boolean m_finished = false;
    private double m_lastUpdated;
    private int m_targetId;

    public AlignAprilTag(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem, double offsetX,
            double offsetY) {
        this(visionSubsystem, swerveSubsystem);
        setOffsetPose(offsetX, offsetY);
    }

    public AlignAprilTag(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem) {
        m_visionSubsystem = visionSubsystem;
        m_swerveSubsystem = swerveSubsystem;

        addRequirements(m_visionSubsystem);
        addRequirements(m_swerveSubsystem);
    }

    @Override
    public void initialize() {
        m_lastUpdated = Timer.getFPGATimestamp();
        m_finished = false;

        Optional<PhotonTrackedTarget> target = m_visionSubsystem.getBestTarget();
        if (target.isEmpty()) {
            m_finished = true;
            return;
        }

        m_targetId = target.get().getFiducialId();
    }

    /**
     * Think of a simple 2D graph where y is height and x is horizontal. To be away
     * from and left of an AprilTag is like quadrant 3 (-,-)
     * 
     * @param x - Positive X moves toward the right of an AprilTag and negative X
     *          goes to the left of an AprilTag.
     * @param y - Positive Y goes past an AprilTag while negative Y retreats
     *          backwards from an AprilTag.
     */
    public Transform2d setOffsetPose(double x, double y) {
        return m_poseOffset = new Transform2d(y, x, Rotation2d.kZero);
    }

    @Override
    public void execute() {
        Logger.recordOutput("connected()", Cameras.LEFT_CAM.camera.isConnected());

        Optional<Pose2d> potentialPose = m_visionSubsystem.getRobotPoseRelativeToAprilTag(m_targetId);

        // If it has been more than a second without seeing a target, stop moving.
        // This fixes issues when the AprilTag is physically out of view but the robot
        // is still set to keep moving forward. I'm praying the delay isn't that bad on
        // the real bot.
        /*
         * if ((Timer.getFPGATimestamp() - m_lastUpdated) > 1) {
         * m_swerveSubsystem.getLibSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
         * m_finished = true;
         * return;
         * }
         * 
         * 
         * 
         */

        if (potentialPose.isEmpty()) {
            m_swerveSubsystem.getLibSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
            return;
        }

        Pose2d pose = potentialPose.get()
                .plus(m_poseOffset);

        if (pose.getX() < .02 && pose.getY() < .02) {
            m_finished = true;
            return;
        }

        // Calculates the propper speed to correctly face the AprilTag (assuming
        // odometry is perfect, because VisionSubsytem::getAprilTagPose doesn't get the
        // actual AprilTag data but it gets where the AprilTag *should* be on the
        // field).
        Rotation2d targetRotation = pose.getRotation().rotateBy(Rotation2d.k180deg);

        double omegaRadiansPerSecond = m_swerveSubsystem.getLibSwerveDrive().swerveController.headingCalculate(
                m_swerveSubsystem.getPose().getRotation().getRadians(),
                targetRotation.getRadians());

        m_swerveSubsystem.getLibSwerveDrive().swerveController.lastAngleScalar = targetRotation.getRadians();

        ChassisSpeeds targetRelativeSpeeds = new ChassisSpeeds(
                pose.getX() * 0.5, // Forward velocity
                pose.getY() * 0.5, // Sideways velocity
                omegaRadiansPerSecond // Rotational velocity
        );

        Logger.recordOutput("speeds", targetRelativeSpeeds);

        m_swerveSubsystem.getLibSwerveDrive().drive(targetRelativeSpeeds);
        m_lastUpdated = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}
