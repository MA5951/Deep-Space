/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.Chassis.MoveBackwards;
import frc.robot.commands.intake.IntakeMovement;

public class AutomaticTakeHatchPanel extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutomaticTakeHatchPanel() {
    addSequential(new IntakeMovement(0.4));
    addSequential(new WaitCommand(0.3));
    addSequential(new MoveBackwards());
    addSequential(new RumbleJoystick(500));
  }
}
