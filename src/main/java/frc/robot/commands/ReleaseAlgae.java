// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeMechanismConstants;
import frc.robot.subsystems.AlgaeSubsystem;

public class ReleaseAlgae extends Command {
    private final AlgaeSubsystem m_algaeSubsystem;

    public ReleaseAlgae(AlgaeSubsystem algaeSubsystem) {
        m_algaeSubsystem = algaeSubsystem;

        addRequirements(m_algaeSubsystem);
    }

    @Override
    public void execute() {
        m_algaeSubsystem.spinMotors(AlgaeMechanismConstants.kReleaseSpeed);
    }
}
