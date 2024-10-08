// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.commands.DriveWithJoystick;

public class RobotContainer {

  public static final CommandXboxController m_controler = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
     DriveWithJoystick telop = new DriveWithJoystick(() -> m_controler.getLeftY(), () -> m_controler.getRightX());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
