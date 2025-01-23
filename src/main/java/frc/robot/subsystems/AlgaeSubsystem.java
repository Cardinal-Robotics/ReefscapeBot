// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.AlgaeMechanismConstants;

public class AlgaeSubsystem extends SubsystemBase {
    private final SparkMax m_followerMotor;
    private final SparkMax m_leadMotor;
    private final SparkMax m_tiltMotor;

    private double m_setpoint;

    public AlgaeSubsystem() {
        m_followerMotor = new SparkMax(AlgaeMechanismConstants.kFollowerMotorPort, MotorType.kBrushed);
        m_leadMotor = new SparkMax(AlgaeMechanismConstants.kLeaderMotorPort, MotorType.kBrushed);
        m_tiltMotor = new SparkMax(AlgaeMechanismConstants.kTiltMotorPort, MotorType.kBrushless);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(IdleMode.kBrake);
        followerConfig.follow(m_leadMotor);
        followerConfig.inverted(true);

        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(IdleMode.kBrake);

        SparkMaxConfig tiltConfig = new SparkMaxConfig();
        tiltConfig.idleMode(IdleMode.kCoast);
        tiltConfig.closedLoop.pid(
                AlgaeMechanismConstants.kTiltKp,
                AlgaeMechanismConstants.kTiltKi,
                AlgaeMechanismConstants.kTiltKd);
        tiltConfig.closedLoop.velocityFF(m_setpoint);

        m_followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_leadMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_tiltMotor.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * @param speed The speed to set. Value should be between -1.0 and 1.0.
     */
    public void spinMotors(double speed) {
        m_leadMotor.set(speed);
        m_followerMotor.set(speed);
    }

    public void stopMotors() {
        spinMotors(0);
    }

    public void setTiltTarget(double setpoint) {
        m_setpoint = setpoint;
        m_tiltMotor.getClosedLoopController().setReference(setpoint, ControlType.kPosition);
    }

    public boolean isTiltMotorAtGoal() {
        double currentPosition = m_tiltMotor.getEncoder().getPosition();
        double error = Math.abs(m_setpoint - currentPosition);
        return error <= 0.2; // Returns true if within the tolerance range
    }

    @Override
    public void periodic() {

    }
}
