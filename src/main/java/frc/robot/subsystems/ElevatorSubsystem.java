// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.ElevatorConstants.ElevatorTarget;
import frc.robot.Constants.ElevatorConstants.InteractionState;

public class ElevatorSubsystem extends SubsystemBase {
    private InteractionState m_interactionState = InteractionState.Algae;
    private SparkMax m_master;
    private SparkMax m_slave; // DO NOT CHANGE TO FOLLOWER

    public ElevatorSubsystem() {
        SparkMaxConfig masterConfig = new SparkMaxConfig();
        masterConfig.idleMode(IdleMode.kBrake);

        masterConfig.absoluteEncoder
                .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
                .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor);

        masterConfig.closedLoop.pid(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorI,
                ElevatorConstants.kElevatorD);

        SparkMaxConfig slaveConfig = new SparkMaxConfig();
        slaveConfig
                .follow(ElevatorConstants.kMasterMotorId)
                .inverted(true)
                .idleMode(IdleMode.kBrake);

        slaveConfig.absoluteEncoder
                .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
                .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor);

        m_master = new SparkMax(ElevatorConstants.kMasterMotorId, MotorType.kBrushless);
        m_slave = new SparkMax(ElevatorConstants.kSlaveMotorId, MotorType.kBrushless);

        m_master.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_slave.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); // Look up
                                                                                                          // later
    }

    public void setInteractionState(InteractionState state) {
        m_interactionState = state;
    }

    public InteractionState getInteractionState() {
        return m_interactionState;
    }

    public void setElevatorGoal(ElevatorTarget goal) {
        double target = ElevatorPositions.kElevatorPositionAlgaeScore; // idle position
        switch (goal) {
            case CoralIntake:
                target = ElevatorPositions.kElevatorPositionCoralIntake;
                break;
            default:
            case AlgaeScore:
                target = ElevatorPositions.kElevatorPositionAlgaeScore;
                break;
            case L1:
                target = (m_interactionState == InteractionState.Algae) ? ElevatorPositions.kElevatorPositionAlgaeL1
                        : ElevatorPositions.kElevatorPositionCoralL1;
                break;
            case L2:
                target = (m_interactionState == InteractionState.Algae) ? ElevatorPositions.kElevatorPositionAlgaeL2
                        : ElevatorPositions.kElevatorPositionCoralL2;
                break;
            case L3:
                target = (m_interactionState == InteractionState.Algae) ? ElevatorPositions.kElevatorPositionAlgaeL3
                        : ElevatorPositions.kElevatorPositionCoralL3;
                break;
            case L4:
                target = (m_interactionState == InteractionState.Algae) ? ElevatorPositions.kElevatorPositionAlgaeL4
                        : ElevatorPositions.kElevatorPositionCoralL4;
                break;
        }
        m_master.getClosedLoopController().setReference(target, ControlType.kPosition);
    }

    public void setElevatorGoal(ElevatorTarget goal, InteractionState state) {
        setInteractionState(state);
        setElevatorGoal(goal);
    }

    public double getPosition() {
        return m_master.getAbsoluteEncoder().getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("ElevatorSubsystem::getPosition()", getPosition());
    }
}
