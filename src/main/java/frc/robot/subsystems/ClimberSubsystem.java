// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final WPI_TalonSRX m_slaveMotor = new WPI_TalonSRX(ClimberConstants.kFollowerMotorId);
    private final WPI_TalonSRX m_masterMotor = new WPI_TalonSRX(ClimberConstants.kLeaderMotorId);
    private final Encoder m_climberEncoder = new Encoder(9, 8);
    private final PIDController m_climberPID = new PIDController(
            ClimberConstants.kClimberP,
            ClimberConstants.kClimberI,
            ClimberConstants.kClimberD);
    double power = 0;

    public ClimberSubsystem() {
        m_slaveMotor.follow(m_masterMotor);
        m_slaveMotor.setInverted(true);
        SmartDashboard.putNumber("ClimberTilt", power);

    }

    public double getClimberPosition() {
        double currentPos = m_climberEncoder.getDistance();
        double degrees = (currentPos / 4096) * 360;

        return degrees;
    }

    public void setGoal(double target) {
        m_climberPID.setSetpoint(target);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("ClimberSubsystem::getClimberPosition", getClimberPosition());
        // setGoal(SmartDashboard.getNumber("ClimberTilt", 0));

        double output = m_climberPID.calculate(getClimberPosition());
        m_masterMotor.set(SmartDashboard.getNumber("ClimberTilt", 0));
    }
}
