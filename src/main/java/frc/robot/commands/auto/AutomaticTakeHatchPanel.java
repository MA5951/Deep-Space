/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.elevator.ElevatorUp;
import frc.robot.commands.rider.MoveAngle;
import frc.robot.commands.rider.RiderMoveToLimitSwitch;

public class AutomaticTakeHatchPanel extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutomaticTakeHatchPanel() {
    addSequential(new ElevatorUp());
    addSequential(new RiderMoveToLimitSwitch());
    addSequential(new AutomaticIntake(-400, -430, -0.5));
    addSequential(new RumbleJoystick(500));
  }
}
