// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.AlgaeMechanismConstants;

public class AlgaeSubsystem extends SubsystemBase {
    private final SparkMax m_leadMotor;
    private final SparkMax m_followerMotor;

    public AlgaeSubsystem() {
        m_leadMotor = new SparkMax(AlgaeMechanismConstants.kLeaderMotorPort, MotorType.kBrushless);
        m_followerMotor = new SparkMax(AlgaeMechanismConstants.kFollowerMotorPort, MotorType.kBrushless);

        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(IdleMode.kBrake);
        m_leadMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(IdleMode.kBrake);
        followerConfig.follow(m_leadMotor);
        followerConfig.inverted(true);
        m_followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * @param speed The speed to set. Value should be between -1.0 and 1.0.
     */
    public void spinMotors(double speed) {
        m_leadMotor.set(speed);
        m_followerMotor.set(speed);
    }
}
