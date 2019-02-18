/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.elevator.ElevatorPID;
import frc.robot.commands.intake.IntakeMoveBall;
import frc.robot.commands.intake.IntakePID;
import frc.robot.commands.rider.RiderIntake;
import frc.robot.commands.rider.RiderPID;

public class AutomaticTakeBall extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutomaticTakeBall() {
    addSequential(new IntakePID(-850, 0.1, 15));
    addSequential(new RiderPID(510, 0.1, 15));
    addSequential(new ElevatorPID(4944, 0.2));
    addSequential(new RumbleJoystick(500));
    addParallel(new IntakeMoveBall(-1.0));
    addParallel(new RiderIntake());
    addSequential(new RumbleJoystick(500));

  }
}
