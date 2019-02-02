/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class PistonCommandGroup extends CommandGroup {

  /**
   * Enables the {PistonForward}, wait 0.5 seconds ({TimedCommand} function) and
   * run the {PistonBack} function.
   */
  public PistonCommandGroup() {

    addSequential(new PistonForward());
    addSequential(new TimedCommand(0.5));
    addSequential(new PistonOff());

  }
}
