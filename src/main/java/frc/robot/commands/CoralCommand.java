// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralCommand extends Command {
    /** Creates a new CoralCommand. */
    CoralSubsystem m_coralSubsystem;
    double targetAngle = 0;

    public CoralCommand(CoralSubsystem CoralSubsystem) {
        addRequirements(CoralSubsystem);
        m_coralSubsystem = CoralSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        targetAngle = m_coralSubsystem.setTarget();
        m_coralSubsystem.alignCoral(targetAngle);
        SmartDashboard.putNumber("Rotation", (m_coralSubsystem.getRot() * 360) - 156);// - 277

        if (RobotContainer.m_operatorController.x().getAsBoolean()) {
            m_coralSubsystem.takeIn();
        } else if (RobotContainer.m_operatorController.b().getAsBoolean()) {
            m_coralSubsystem.moveOut();
        } else {
            m_coralSubsystem.stop();
        }
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
