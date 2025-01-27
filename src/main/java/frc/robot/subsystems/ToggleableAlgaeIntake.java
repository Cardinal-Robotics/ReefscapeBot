// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.AlgaeMechanismConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class ToggleableAlgaeIntake extends Command {
    private final AlgaeSubsystem m_algaeSubsystem;
    private final Timer m_timer = new Timer();
    private boolean m_hasAlgae = false;
    private AlgaeState m_currentState;

    public enum AlgaeState {
        ELEVATOR_AT_PROCESSOR,
        TILT_TO_RELEASE,
        TILT_TO_INTAKE,
        TILT_TO_DRIVE,
        RELEASING,
        INTAKING,
        IDLE
    }

    public ToggleableAlgaeIntake(AlgaeSubsystem algaeSubsystem) {
        m_currentState = m_hasAlgae ? AlgaeState.TILT_TO_RELEASE : AlgaeState.TILT_TO_INTAKE;

        m_algaeSubsystem = algaeSubsystem;
        addRequirements(m_algaeSubsystem);
    }

    @Override
    public void initialize() {
        // Set the starting state based on whether algae is currently held
        m_currentState = m_hasAlgae ? AlgaeState.TILT_TO_RELEASE : AlgaeState.TILT_TO_INTAKE;

        m_timer.reset();
        m_timer.stop();
    }

    @Override
    public void execute() {
        switch (m_currentState) {
            case TILT_TO_INTAKE: // Tilts the algae mechanism downwards so that it can properly pull the algae
                                 // out of the reef.
                m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetPointIntake);
                if (m_algaeSubsystem.isTiltMotorAtGoal(AlgaeMechanismConstants.kTargetPointIntake)) {
                    m_currentState = AlgaeState.INTAKING;
                    m_timer.reset();
                    m_timer.start();
                }
                break;

            case INTAKING: // Pulls the algae out of the reef.
                m_algaeSubsystem.spinIntakeMotor(AlgaeMechanismConstants.kIntakeSpeed);
                if (m_timer.hasElapsed(AlgaeMechanismConstants.kIntakeTime)) {
                    m_algaeSubsystem.stopIntakeMotor();
                    m_timer.reset();
                    m_timer.stop();
                    m_currentState = AlgaeState.TILT_TO_DRIVE;
                }
                break;

            case TILT_TO_DRIVE: // Moves the algae mechanism to the driving state so it can be driven around
                                // with.
                m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetPointDrive);
                if (m_algaeSubsystem.isTiltMotorAtGoal(AlgaeMechanismConstants.kTargetPointDrive)) {
                    m_currentState = AlgaeState.IDLE;
                    m_hasAlgae = !m_hasAlgae; // Toggle the algae state
                }
                break;

            case TILT_TO_RELEASE: // Makes sure that the algae mechanism is faced downwards at the processor so
                                  // the ball can make it through.
                m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetPointRelease);
                // TODO: Call elevator to processor setpoint early (in this block of code) so
                // that it both tilts and does Elevator PID so as to not waste any time.

                if (m_algaeSubsystem.isTiltMotorAtGoal(AlgaeMechanismConstants.kTargetPointRelease)) {
                    m_currentState = AlgaeState.ELEVATOR_AT_PROCESSOR;
                    m_timer.reset();
                    m_timer.start();
                }
                break;

            case ELEVATOR_AT_PROCESSOR: // Ensures that the elevator is low enough before releasing the algae.
                m_currentState = AlgaeState.RELEASING;
                break;

            case RELEASING: // Shoots the algae ball out of the algae mechanism.
                m_algaeSubsystem.spinIntakeMotor(AlgaeMechanismConstants.kReleaseSpeed);
                if (m_timer.hasElapsed(AlgaeMechanismConstants.kReleaseTime)) {
                    m_algaeSubsystem.stopIntakeMotor();
                    m_timer.stop();
                    m_currentState = AlgaeState.TILT_TO_DRIVE;
                }
                break;

            case IDLE: // The robot has finished either intaking or releasing so it ends the command.
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return m_currentState == AlgaeState.IDLE;
    }
}
