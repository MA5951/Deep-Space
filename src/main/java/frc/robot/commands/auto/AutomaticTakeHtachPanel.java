/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.elevator.ElevatorUp;

public class AutomaticTakeHtachPanel extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutomaticTakeHtachPanel() {
    addSequential(new ElevatorUp());
    //TODO Move rider to the back
    addSequential(new AutomaticIntake(-400, -430 , -0.5));
  }
}
