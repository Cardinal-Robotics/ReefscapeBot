// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visual;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util;

public class GenericMotorVisual extends SubsystemBase {
    private boolean challengeStarted = false;
    private Timer timer = new Timer();
    private SparkMaxSim m_motorSim;
    private SparkMax m_motor;
    private double m_angle;

    private final Mechanism2d mech = new Mechanism2d(3, 3);
    private final MechanismRoot2d root = mech.getRoot("Motor", 1.5, 1.5);
    private final MechanismLigament2d wheel = new MechanismLigament2d("Wheel", 1.0, 0);
    private final MechanismLigament2d opposingWheel = wheel.append(new MechanismLigament2d("Wheel", -1.0, 0));

    private final LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1),
            0.00096,
            1.0);
    private final FlywheelSim flywheelSim = new FlywheelSim(flywheelPlant, DCMotor.getNEO(1));

    public GenericMotorVisual(SparkMaxSim motorSim, SparkMax motor) {
        m_motorSim = motorSim;
        m_motor = motor;
        initialize();
    }

    public GenericMotorVisual(SparkMaxSim motorSim) {
        m_motorSim = motorSim;
        m_motor = Util.getMotorFromSim(motorSim);
        initialize();
    }

    public void startChallengeMode() {
        if (challengeStarted)
            return;
        challengeStarted = true;
        timer.start();
    }

    public boolean isChallengeModeReady() {
        return timer.hasElapsed(3);
    }

    public void initialize() {
        root.append(wheel);
        SmartDashboard.putData("Motor Visualization", mech);
    }

    @Override
    public void simulationPeriodic() {
        double dt = 0.02;
        double busVoltage = m_motorSim.getBusVoltage();

        // during challenge spin-up, force open-loop control
        if (challengeStarted && !isChallengeModeReady()) {
            m_motor.set(-1.0);
        }

        // 1. Run REVSim first — this updates internal PID output
        // feed the *current* flywheel speed and voltage estimate
        double velocityRPM = Units.radiansPerSecondToRotationsPerMinute(flywheelSim.getAngularVelocityRadPerSec());
        m_motorSim.iterate(velocityRPM, busVoltage, dt);

        // 2. Now read what the PID decided to do
        double appliedOutput = m_motorSim.getAppliedOutput();
        double motorVoltage = appliedOutput * busVoltage;

        // 3. Advance the flywheel physics with that voltage
        flywheelSim.setInputVoltage(motorVoltage);
        flywheelSim.update(dt);

        // 4. Integrate the position (radians → rotations)
        m_angle += flywheelSim.getAngularVelocityRadPerSec() * dt;
        m_motor.getEncoder().setPosition(m_angle / (2.0 * Math.PI));

        // 5. Update the mechanism visual
        wheel.setAngle(Math.toDegrees(m_angle));

        // 6. Telemetry
        SmartDashboard.putNumber("Encoder RPM (reported)", m_motor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Applied Output (sim)", m_motorSim.getAppliedOutput()); // -1..1
        SmartDashboard.putNumber("Bus Voltage (V)", m_motorSim.getBusVoltage());
        SmartDashboard.putNumber("Motor Voltage (V)", m_motorSim.getAppliedOutput() * m_motorSim.getBusVoltage());
        SmartDashboard.putNumber("FlywheelSim RPM", flywheelSim.getAngularVelocityRPM());
    }
}
