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
import edu.wpi.first.wpilibj.Timer;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Robot;

import java.util.Optional;

public class AlignAprilTag extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    private final SwerveSubsystem m_swerveSubsystem;

    private final PIDController xTranslationController = new PIDController(1, 0, 0);
    private final PIDController yTranslationController = new PIDController(5, 0, 0);

    private Transform2d m_poseOffset = setOffsetPose(0, -0.5);
    private boolean m_finished = false;
    private double m_lastUpdated;
    private int m_targetId;

    public AlignAprilTag(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem,
            ElevatorSubsystem elevatorSubsystem, double offsetX,
            double offsetY) {
        this(visionSubsystem, swerveSubsystem, elevatorSubsystem);
        setOffsetPose(offsetX, offsetY);
    }

    public AlignAprilTag(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem,
            ElevatorSubsystem elevatorSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_visionSubsystem = visionSubsystem;
        m_swerveSubsystem = swerveSubsystem;

        addRequirements(m_visionSubsystem);
        addRequirements(m_swerveSubsystem);

        SmartDashboard.putNumber("customAlignX", 0);
        SmartDashboard.putNumber("customAlignY", 0);
    }

    @Override
    public void initialize() {
        m_lastUpdated = Timer.getFPGATimestamp();
        m_finished = false;

        Optional<PhotonTrackedTarget> target = Robot.isSimulation() ? m_visionSubsystem.getClosestTarget()
                : m_visionSubsystem.getBestTarget();
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
        // m_poseOffset = new Transform2d(SmartDashboard.getNumber("customAlignX", 0),
        // SmartDashboard.getNumber("customAlignY", 0), Rotation2d.kZero);
        Optional<Transform2d> potentialPose = m_visionSubsystem.getRobotPoseRelativeToAprilTag(m_targetId);
        Logger.recordOutput("K", potentialPose.isPresent() ? potentialPose.get() : Transform2d.kZero);

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

        Transform2d pose = potentialPose.get();
        if (pose == null)
            return;

        Logger.recordOutput("Offset pose",
                new Pose2d(m_swerveSubsystem.getPose().getTranslation().plus(pose.getTranslation()).getX(),
                        m_swerveSubsystem.getPose().getTranslation().plus(pose.getTranslation()).getY(),
                        Rotation2d.kZero));

        /*
         * if (pose.getX() < .02 && pose.getY() < .02) {
         * m_finished = true;
         * return;
         * }
         */

        // Calculates the propper speed to correctly face the AprilTag (assuming
        // odometry is perfect, because VisionSubsytem::getAprilTagPose doesn't get the
        // actual AprilTag data but it gets where the AprilTag *should* be on the
        // field).
        Rotation2d targetRotation = pose.getRotation().rotateBy(Rotation2d.k180deg);

        double omegaRadiansPerSecond = m_swerveSubsystem.getLibSwerveDrive().swerveController.headingCalculate(
                m_swerveSubsystem.getPose().getRotation().getRadians(),
                targetRotation.getRadians());

        m_swerveSubsystem.getLibSwerveDrive().swerveController.lastAngleScalar = targetRotation.getRadians();

        double xTranslationSpeed = -xTranslationController.calculate(pose.getX(), m_poseOffset.getX());
        double yTranslationSpeed = -yTranslationController.calculate(pose.getY(), m_poseOffset.getY());
        double speedLimiter = Math.max((1 - (m_elevatorSubsystem.getPosition() / 1.3)), 0.1);

        ChassisSpeeds targetRelativeSpeeds = new ChassisSpeeds(
                MathUtil.clamp(xTranslationSpeed, -speedLimiter, speedLimiter), // Forward velocity
                MathUtil.clamp(yTranslationSpeed, -speedLimiter, speedLimiter), // Sideways velocity
                omegaRadiansPerSecond // Rotational velocity
        );

        Logger.recordOutput("AT Align X Error", xTranslationController.getError());
        Logger.recordOutput("AT Align Y Error", yTranslationController.getError());

        m_swerveSubsystem.getLibSwerveDrive().drive(targetRelativeSpeeds);

        m_lastUpdated = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}
