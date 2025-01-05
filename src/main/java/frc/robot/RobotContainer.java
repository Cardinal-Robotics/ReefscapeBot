// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    private final XboxController m_driverController = new XboxController(
            OperatorConstants.kDriverControllerPort);
    private final XboxController m_operatorController = new XboxController(
            OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

    }

    public Command getAutonomousCommand() {
        return null;
    }
}
