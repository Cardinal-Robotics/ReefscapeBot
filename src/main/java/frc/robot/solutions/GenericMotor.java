// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.solutions;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.visual.GenericMotorVisual;

public class GenericMotor extends SubsystemBase {
    private SparkMax motor = new SparkMax(0, MotorType.kBrushless);
    private SparkMaxSim motorSim = new SparkMaxSim(motor, DCMotor.getNEO(1));

    private GenericMotorVisual visual = new GenericMotorVisual(motorSim);

    private boolean challengeMode = true;

    public GenericMotor() {
        SmartDashboard.putNumber("Set Motor Speed", 0);
        SmartDashboard.putNumber("Rotation Target", -30);
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(0.5, 0, 0.001);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        if (challengeMode)
            visual.startChallengeMode();
    }

    @Override
    public void periodic() {
        double speed = SmartDashboard.getNumber("Set Motor Speed", 0);

        if (visual.isChallengeModeReady()) {
            double target = SmartDashboard.getNumber("Rotation Target", -30);
            motor.getClosedLoopController().setReference(target / 360, ControlType.kPosition);
        } else {
            motor.set(speed);
        }
    }
}
