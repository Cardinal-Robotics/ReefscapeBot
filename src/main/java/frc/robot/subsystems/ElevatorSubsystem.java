// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.ElevatorConstants.ElevatorTarget;
import frc.robot.RobotContainer.InteractionState;
import frc.robot.Constants.CoralMechanismConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotContainer;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;

import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_master = new SparkMax(ElevatorConstants.kMasterMotorId, MotorType.kBrushless);
    // DO NOT CHANGE TO FOLLOWER
    private final SparkMax m_slave = new SparkMax(ElevatorConstants.kSlaveMotorId, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_master.getEncoder();
    private CoralSubsystem m_coralSubsystem = null;

    // Feedforward stuff (if needed)

    /*
     * private final TrapezoidProfile m_profile = new TrapezoidProfile(new
     * TrapezoidProfile.Constraints(1.75, 0.75));
     * private final ElevatorFeedforward m_elevatorFeedforward = new
     * ElevatorFeedforward(
     * ElevatorConstants.kElevatorKs,
     * ElevatorConstants.kElevatorKg,
     * ElevatorConstants.kElevatorKv);
     */

    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    private ElevatorSim m_elevatorSim;
    private SparkMaxSim m_motorSim;

    public double getTarget() {
        return m_setpoint.position;
    }

    public boolean atTarget() {
        return (getPosition() < getTarget() + .1) && (getPosition() > getTarget() - .1);
    }

    @Override
    public void periodic() { // 49 + 3/8
        SmartDashboard.putNumber("master motor output %", m_master.getAppliedOutput());
        SmartDashboard.putNumber("ElevatorSubsystem::getPosition", getPosition());

        double targetPosition = m_setpoint.position;

        if (m_coralSubsystem == null || !m_coralSubsystem.atTarget(CoralMechanismConstants.kTargetAngleStore))
            targetPosition = getPosition();

        if (DriverStation.isDisabled())
            m_setpoint = new TrapezoidProfile.State(getPosition(), 0);

        // If the elevator is applying 50% or more of its speed but it's not moving,
        // stop. You are probably crushing the coral mechanism.

        /*
         * if (m_master.getAppliedOutput() > 0.5 && m_encoder.getVelocity() < 0.1 &&
         * !Robot.isSimulation())
         * targetPosition = getPosition();
         */

        m_master.getClosedLoopController().setReference(targetPosition, ControlType.kPosition);

        /*
         * m_setpoint = m_profile.calculate(0.02,
         * new TrapezoidProfile.State(getPosition(),
         * m_master.getEncoder().getVelocity()), m_goal);
         * m_master.getClosedLoopController().setReference(
         * m_setpoint.position,
         * ControlType.kPosition,
         * ClosedLoopSlot.kSlot0,
         * m_elevatorFeedforward.calculate(m_setpoint.velocity));
         */
    }

    @Override
    public void simulationPeriodic() {
        m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_motorSim.iterate(m_elevatorSim.getVelocityMetersPerSecond(), RoboRioSim.getVInVoltage(), 0.02);
        m_elevatorSim.update(0.020);

        m_master.getClosedLoopController().setReference(SmartDashboard.getNumber("ElevatorHeight", 0),
                ControlType.kPosition);

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }

    /**
     * Index 0 returns the position of the inner stage while index 1 returns the
     * position of the middle stage
     */
    public Pose3d[] getSimulatedElevatorPositions() {
        double middleStageHight = m_encoder.getPosition() / (0.6009005308151245 + Units.inchesToMeters(24 + (1 / 16)))
                * 0.6009005308151245;
        double innerStageHight = m_encoder.getPosition();

        return new Pose3d[] {
                new Pose3d(0, 0, middleStageHight, Rotation3d.kZero),
                new Pose3d(0, 0, innerStageHight, Rotation3d.kZero)
        };
    }

    public ElevatorSubsystem(CoralSubsystem coralSubsystem) {
        SmartDashboard.putNumber("ElevatorHeight", 0);
        SparkMaxConfig masterConfig = new SparkMaxConfig();
        masterConfig.idleMode(IdleMode.kCoast);
        m_coralSubsystem = coralSubsystem;

        masterConfig.encoder.positionConversionFactor(ElevatorConstants.kPositionConversionFactor);
        masterConfig.encoder.velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor);

        masterConfig.closedLoop
                .pid(ElevatorConstants.kElevatorP,
                        ElevatorConstants.kElevatorI,
                        ElevatorConstants.kElevatorD)
                .outputRange(-.7, .7);

        SparkMaxConfig slaveConfig = new SparkMaxConfig();
        slaveConfig
                .follow(ElevatorConstants.kMasterMotorId, true)
                .idleMode(IdleMode.kCoast);

        slaveConfig.encoder.positionConversionFactor(ElevatorConstants.kPositionConversionFactor);

        m_master.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_slave.configure(slaveConfig, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters); // Look up later

        // Simulation code
        if (!Robot.isSimulation())
            return;

        m_motorSim = new SparkMaxSim(m_master, DCMotor.getNEO(2));

        m_elevatorSim = new ElevatorSim(DCMotor.getNEO(2),
                20,
                18.0,
                Units.inchesToMeters(1.432) / 2,
                0,
                1.3,
                false,
                0,
                0.001,
                0.0);
    }

    // Getters & setters
    public void setCoralSubsystem(CoralSubsystem coralSubsystem) {
        this.m_coralSubsystem = coralSubsystem;
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public void setElevatorGoal(ElevatorTarget goal, InteractionState state) {
        RobotContainer.interactionState = state;
        setElevatorGoal(goal);
    }

    public void setElevatorGoal(ElevatorTarget targetIfCoral, ElevatorTarget targetIfAlgae) {
        if (RobotContainer.interactionState == InteractionState.Coral)
            setElevatorGoal(targetIfCoral);
        else if (RobotContainer.interactionState == InteractionState.Algae)
            setElevatorGoal(targetIfAlgae);
    }

    // --- Handles elevator target
    public void setElevatorGoal(ElevatorTarget goal) {
        double target = RobotContainer.interactionState == InteractionState.Coral ? goal.getCoralPosition()
                : goal.getAlgaePosition();

        if (target == Double.NaN)
            target = m_setpoint.position;

        m_setpoint = new TrapezoidProfile.State(target, 0);
        SmartDashboard.putNumber("ElevatorHeight", target);
    }

    public Command setElevatorGoalCommand(ElevatorTarget goal) {
        return run(() -> setElevatorGoal(goal)).until(() -> atTarget());
    }
}
