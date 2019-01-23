/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class PistonBack extends InstantCommand {

  private Intake intake = Intake.getInstance();

  /**
   * requires the Intale subsystem.
   */
  public PistonBack() {
    requires(intake);
  }

  /**
   * Run the {RelayControlRevers} function.
   */
  @Override
  protected void initialize() {
    intake.RelayControlReverse();
  }

}
