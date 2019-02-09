/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.elevator.ElevatorUp;
import frc.robot.commands.intake.IntakeMovement;
import frc.robot.commands.rider.MoveToLimitSwitch;

public class ReturnToDefault extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ReturnToDefault() {
    addSequential(new ElevatorUp());
    addSequential(new MoveToLimitSwitch());
    addSequential(new IntakeMovement(0.5));
    addSequential(new RumbleJoystick(500));
  }
}