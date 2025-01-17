// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.parser.PIDFConfig;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UpdatePIDF extends Command {
    /** Creates a new UpdatePIDF. */
    private final SwerveSubsystem m_swerveSubsystem;

    public UpdatePIDF(SwerveSubsystem subsystem) {
        m_swerveSubsystem = subsystem;
        addRequirements(m_swerveSubsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putNumber("Drive P", m_swerveSubsystem.getLibSwerveDrive().getModules()[0].getDrivePIDF().p);
        SmartDashboard.putNumber("Drive I", m_swerveSubsystem.getLibSwerveDrive().getModules()[0].getDrivePIDF().i);
        SmartDashboard.putNumber("Drive D", m_swerveSubsystem.getLibSwerveDrive().getModules()[0].getDrivePIDF().d);
        SmartDashboard.putNumber("Drive F", m_swerveSubsystem.getLibSwerveDrive().getModules()[0].getDrivePIDF().f);
        SmartDashboard.putNumber("Drive iz", m_swerveSubsystem.getLibSwerveDrive().getModules()[0].getDrivePIDF().iz);

        SmartDashboard.putNumber("Angular P", m_swerveSubsystem.getLibSwerveDrive().getModules()[0].getAnglePIDF().p);
        SmartDashboard.putNumber("Angular I", m_swerveSubsystem.getLibSwerveDrive().getModules()[0].getAnglePIDF().i);
        SmartDashboard.putNumber("Angular D", m_swerveSubsystem.getLibSwerveDrive().getModules()[0].getAnglePIDF().d);
        SmartDashboard.putNumber("Angular F", m_swerveSubsystem.getLibSwerveDrive().getModules()[0].getAnglePIDF().f);
        SmartDashboard.putNumber("Angular iz", m_swerveSubsystem.getLibSwerveDrive().getModules()[0].getAnglePIDF().iz);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_swerveSubsystem.updatePIDF(
                new PIDFConfig(
                        SmartDashboard.getNumber("Drive P", 0),
                        SmartDashboard.getNumber("Drive I", 0),
                        SmartDashboard.getNumber("Drive D", 0),
                        SmartDashboard.getNumber("Drive F", 0),
                        SmartDashboard.getNumber("Drive iz", 0)),
                new PIDFConfig(
                        SmartDashboard.getNumber("Angular P", 0),
                        SmartDashboard.getNumber("Angular I", 0),
                        SmartDashboard.getNumber("Angular D", 0),
                        SmartDashboard.getNumber("Angular F", 0),
                        SmartDashboard.getNumber("Angular iz", 0))

        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
