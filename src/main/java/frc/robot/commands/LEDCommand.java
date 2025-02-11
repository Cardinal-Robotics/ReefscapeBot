// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LEDCommand extends Command {
    /** Creates a new LEDCommand. */
    LEDSubsystem m_LEDSubsystem;
    private float valor = 0;

    public LEDCommand(LEDSubsystem LEDSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(LEDSubsystem);
        m_LEDSubsystem = LEDSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.m_operatorController.rightBumper().getAsBoolean()) {
            valor = m_LEDSubsystem.increaseValor(valor);
        } else if (RobotContainer.m_operatorController.leftBumper().getAsBoolean()) {
            valor = m_LEDSubsystem.decreaseValor(valor);
        }

        if (RobotContainer.m_operatorController.b().getAsBoolean()) {
            m_LEDSubsystem.setBlue();
        } else if (RobotContainer.m_operatorController.x().getAsBoolean()) {
            m_LEDSubsystem.setRainbow();
        } else if (RobotContainer.m_operatorController.y().getAsBoolean()) {
            m_LEDSubsystem.elevatorPattern(valor);
        } else if (RobotContainer.m_operatorController.rightStick().getAsBoolean()) {
            m_LEDSubsystem.setChoppedRainbow();
        } else {
            m_LEDSubsystem.setRed();
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
