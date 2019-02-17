/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class AllAutomaticIntake extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AllAutomaticIntake() {
    addSequential(new AutomaticTakeBall());
    addSequential(new WaitCommand(0.5));
    addSequential(new ReturnToDefault());
    addSequential(new RumbleJoystick(500));
  }
}
