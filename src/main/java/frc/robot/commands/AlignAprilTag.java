// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
        addRequirements();
    }

    @Override
    public void execute() {
        double x = m_limeLightSubsystem.getTX();
        double y = m_limeLightSubsystem.getTY();

        SmartDashboard.putNumber("Tag x-offset", x); // logs values to make sure we get them
        SmartDashboard.putNumber("Tag y-offset", y);

        // this aligns first with x then goes with y once x aligned
        if (x > 2) { // moves robot right
            m_swerveSubsystem.driveRelative(0, 0.8, 0);
        } else if (x < -2) { // left
            m_swerveSubsystem.driveRelative(0, -0.8, 0);
        }

        if (x < 2 && x > -2) { // once x aligned, y
            if (y < 11) { // forward
                m_swerveSubsystem.driveRelative(-0.8, 0, 0);
            } else if (y > 15) { // back
                m_swerveSubsystem.driveRelative(0.8, 0, 0);
            }
        }
    }
}
