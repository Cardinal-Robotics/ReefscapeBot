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
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.AlgaeMechanismConstants;

public class AlgaeSubsystem extends SubsystemBase {
    private final SparkMax m_intakeMotor;
    private final SparkMax m_tiltMotor;

    private double m_setpoint;

    public AlgaeSubsystem() {
        m_intakeMotor = new SparkMax(AlgaeMechanismConstants.kIntakeMotorPort, MotorType.kBrushed);
        m_tiltMotor = new SparkMax(AlgaeMechanismConstants.kTiltMotorPort, MotorType.kBrushless);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake);

        SparkMaxConfig tiltConfig = new SparkMaxConfig();
        tiltConfig.idleMode(IdleMode.kCoast);
        tiltConfig.closedLoop.pid(
                AlgaeMechanismConstants.kTiltKp,
                AlgaeMechanismConstants.kTiltKi,
                AlgaeMechanismConstants.kTiltKd);
        tiltConfig.closedLoop.velocityFF(AlgaeMechanismConstants.kTiltFeedForward);

        m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_tiltMotor.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * @param speed The speed to set. Value should be between -1.0 and 1.0.
     */
    public void spinMotors(double speed) {
        m_intakeMotor.set(speed);
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
}
