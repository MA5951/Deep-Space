/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Elevator;

/**
 * Add your docs here.
 */
public class ResetElevatorEncoder extends InstantCommand {
  Elevator elevator = Elevator.getInstance();



  public ResetElevatorEncoder() {
    requires(elevator);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    elevator.resetEncoder();
  }

}
