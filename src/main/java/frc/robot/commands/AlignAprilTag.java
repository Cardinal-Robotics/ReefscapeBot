// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignAprilTag extends Command {
    private final LimelightSubsystem m_limeLightSubsystem;
    private final SwerveSubsystem m_swerveSubsystem;

    public AlignAprilTag(LimelightSubsystem limeLightSubsystem, SwerveSubsystem swerveSubsystem) {
        m_limeLightSubsystem = limeLightSubsystem;
        m_swerveSubsystem = swerveSubsystem;

        addRequirements(m_limeLightSubsystem);
        // Don't require the swerve subsystem, it causes YAGSL to freak out. At least in
        // the simulation.
        // addRequirements(m_swerveSubsystem);
    }

    @Override
    public void execute() {
        Pose2d pose = m_limeLightSubsystem.getTagPoseRelativeToRobot();

        if (pose.getX() == 0 && pose.getY() == 0 && pose.getRotation().getDegrees() == 0)
            return;

        m_swerveSubsystem.driveCustomPoseOriented(
                new Pose2d(Translation2d.kZero, Rotation2d.fromDegrees(-60)),
                -pose.getY(),
                -pose.getX(), -pose.getRotation().getDegrees());

        // this aligns first with x then goes with y once x aligned
        /*
         * if (y > 2) { // moves robot right
         * m_swerveSubsystem.driveRelative(0, 0.8, 0);
         * } else if (y < -2) { // left
         * m_swerveSubsystem.driveRelative(0, -0.8, 0);
         * }
         * 
         * // If x and y aren't aligned yet, don't align the closeness yet.
         * if (!(y < 2 && y > -2))
         * return;
         * 
         * if (x < 0.5) { // back away from the AprilTag
         * m_swerveSubsystem.driveRelative(-0.8, 0, 0);
         * } else if (x > 0.6) { // move to the AprilTag
         * m_swerveSubsystem.driveRelative(0.8, 0, 0);
         * }
         */

    }
}
