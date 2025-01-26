// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AlgaeMechanismConstants;

public class ToggleableAlgaeIntake extends Command {
    private final AlgaeSubsystem m_algaeSubsystem;
    private boolean m_hasAlgae;

    public ToggleableAlgaeIntake(AlgaeSubsystem algaeSubsystem) {
        m_algaeSubsystem = algaeSubsystem;

        /*
         * new StartEndCommand(
         * () -> Commands.run(() -> System.out.println("e"),
         * m_algaeSubsystem).andThen(new WaitCommand(4))
         * .andThen(() ->
         * m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetPointIntake))
         * .until(() -> m_algaeSubsystem.isTiltMotorAtGoal())
         * .andThen(() ->
         * m_algaeSubsystem.spinMotors(AlgaeMechanismConstants.kIntakeSpeed))
         * .andThen(new WaitCommand(AlgaeMechanismConstants.kIntakeTime))
         * .andThen(() -> m_algaeSubsystem.stopMotors())
         * .andThen(() ->
         * m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetPointDrive))
         * .until(() -> m_algaeSubsystem.isTiltMotorAtGoal())
         * .schedule(),
         * () -> Commands.run(() ->
         * m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetPointRelease),
         * m_algaeSubsystem)
         * .until(() -> m_algaeSubsystem.isTiltMotorAtGoal())
         * .andThen(() ->
         * m_algaeSubsystem.spinMotors(AlgaeMechanismConstants.kReleaseSpeed))
         * .andThen(new WaitCommand(AlgaeMechanismConstants.kReleaseTime))
         * .andThen(() -> m_algaeSubsystem.stopMotors())
         * .andThen(() ->
         * m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetPointDrive))
         * .until(() -> m_algaeSubsystem.isTiltMotorAtGoal())
         * .schedule());
         */
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
