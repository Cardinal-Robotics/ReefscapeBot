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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    SparkMax m_master;
    SparkMax m_slave; // DO NOT CHANGE TO FOLLOWER

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
        m_slave.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); // Look up
                                                                                                          // later

    }

    public void setElevatorGoal(double setPoint) {
        m_master.getClosedLoopController().setReference(setPoint, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
