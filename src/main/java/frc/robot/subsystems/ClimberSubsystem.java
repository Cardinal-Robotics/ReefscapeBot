// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private WPI_TalonSRX m_slaveMotor = new WPI_TalonSRX(ClimberConstants.kFollowerMotorId);
    private WPI_TalonSRX m_masterMotor = new WPI_TalonSRX(ClimberConstants.kLeaderMotorId);

    DutyCycleEncoder climberEncoder = new DutyCycleEncoder(ClimberConstants.kEncoderId);

    PIDController climberPID = new PIDController(ClimberConstants.kClimberP, ClimberConstants.kClimberI,
            ClimberConstants.kClimberD);

    public ClimberSubsystem() {
        m_slaveMotor.follow(m_masterMotor);
        m_slaveMotor.setInverted(true);

    }

    public double getClimberPosition() {
        double currentPos = climberEncoder.get();
        double degrees = (currentPos / 4096) * 360;

        return degrees;
    }

    public void setGoal(double target) {
        climberPID.setSetpoint(target);
    }

    public double runPID() {
        return climberPID.calculate(getClimberPosition());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ClimberSubsystem", getClimberPosition());

        double output = runPID();
        // m_masterMotor.set(output);
    }
}
