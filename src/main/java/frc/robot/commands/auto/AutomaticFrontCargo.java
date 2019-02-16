/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.elevator.ElevatorUp;
import frc.robot.commands.intake.IntakePID;
import frc.robot.commands.rider.RiderPID;

public class AutomaticFrontCargo extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutomaticFrontCargo() {
    addSequential(new ElevatorUp());
    addSequential(new IntakePID(-800, 0.1, 15));
    addSequential(new RiderPID(-990, 0.1, 15));
    addSequential(new IntakePID(0, 0.1, 15));
    addSequential(new RumbleJoystick(500));
  }
}
