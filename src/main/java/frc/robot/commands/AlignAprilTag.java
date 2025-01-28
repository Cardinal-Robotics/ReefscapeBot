// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
        if (!m_limeLightSubsystem.hasTargets())
            return;

        double x = m_limeLightSubsystem.getRedX();
        double y = m_limeLightSubsystem.getRedY();
        double angle = m_limeLightSubsystem.getYaw();
        double rotation = m_limeLightSubsystem.alignYaw();

        SmartDashboard.putNumber("Tag x-offset", x); // logs values to make sure we get them
        SmartDashboard.putNumber("Tag y-offset", y);

        // this aligns first with x then goes with y once x aligned
        if (y > 3.4) { // moves robot right //|| angle > -88 || angle < -92
            m_swerveSubsystem
                    .driveFieldOriented(new ChassisSpeeds(-.8, 0,
                            rotation)); // Math.toRadians(180 -
                                        // m_swerveSubsystem.getRotation().getDegrees()))
        } else if (y < 3.2) { // left
            m_swerveSubsystem
                    .driveFieldOriented(new ChassisSpeeds(.8, 0,
                            rotation));
        }

        // If x and y aren't aligned yet, don't align the closeness yet.
        if (!(x < 2.8 && x > 3.3))
            return;
        /*
         * if (y < 11) { // back away from the AprilTag
         * m_swerveSubsystem.driveRelative(0.8, 0, 0);
         * } else if (y > 15) { // move to the AprilTag
         * m_swerveSubsystem.driveRelative(-0.8, 0, 0);
         * }
         */

    }
}
