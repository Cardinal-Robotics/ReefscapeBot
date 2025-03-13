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

import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.subsystems.VisionSubsystem;
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
     * @param x - Positive X goes past an AprilTag while negative X retreats
     *          backwards from an AprilTag.
     * @param y - Positive Y moves toward the right of an AprilTag and negative Y
     *          goes to the left of an AprilTag.
     */
    public Transform2d setOffsetPose(double x, double y) {
        return m_poseOffset = new Transform2d(y, -x, Rotation2d.kZero).inverse();
    }

    @Override
    public void execute() {
        Optional<Pose2d> potentialPose = m_visionSubsystem.getRobotPoseRelativeToAprilTag(m_targetId);

        // If it has been more than a second without seeing a target, stop moving.
        // This fixes issues when the AprilTag is physically out of view but the robot
        // is still set to keep moving forward. I'm praying the delay isn't that bad on
        // the real bot.
        if ((Timer.getFPGATimestamp() - m_lastUpdated) > 1) {
            m_swerveSubsystem.getLibSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
            m_finished = true;
            return;
        }

        if (potentialPose.isEmpty())
            return;

        Pose2d pose = potentialPose.get()
                .plus(m_poseOffset);

        if (pose.getX() < 0.1 && pose.getY() < 0.1) {
            m_finished = true;
            return;
        }

        // Calculates the propper speed to correctly face the AprilTag (assuming
        // odometry is perfect, because VisionSubsytem::getAprilTagPose doesn't get the
        // actual AprilTag data but it gets where the AprilTag *should* be on the
        // field).
        Rotation2d targetRotation = VisionSubsystem.getAprilTagPose(m_targetId, Transform2d.kZero).getRotation()
                .rotateBy(Rotation2d.k180deg) // Gets the opposite direction of the tag so that the robot faces the tag.
                .rotateBy(DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? Rotation2d.k180deg
                        : Rotation2d.kZero); // Flipping alliance stuff

        double omegaRadiansPerSecond = m_swerveSubsystem.getLibSwerveDrive().swerveController.headingCalculate(
                m_swerveSubsystem.getPose().getRotation()
                        .rotateBy(DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? Rotation2d.k180deg
                                : Rotation2d.kZero)
                        .getRadians(), // More alliance flipping.
                targetRotation.getRadians());

        m_swerveSubsystem.getLibSwerveDrive().swerveController.lastAngleScalar = targetRotation.getRadians();

        ChassisSpeeds targetRelativeSpeeds = new ChassisSpeeds(
                pose.getX(), // Forward velocity
                pose.getY(), // Sideways velocity
                omegaRadiansPerSecond // Rotational velocity
        );

        m_swerveSubsystem.getLibSwerveDrive().drive(targetRelativeSpeeds);
        m_lastUpdated = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}
