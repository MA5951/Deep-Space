/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
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
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    addSequential(new IntakePID(-830, 0.1, 15));
    addSequential(new RiderPID(-570, 0.1, 15));
    addSequential(new ElevatorPID(-6310, 0.2));
    addSequential(new WaitCommand(0.1));
    addSequential(new RumbleJoystick(50));
    addParallel(new IntakeMoveBall(-1.0));
    addParallel(new RiderIntake());
    addSequential(new RumbleJoystick(50));

    // addSequential(new MoveAngle(0, -20, -0.3));
    // addSequential (new IntakeMovement(0.5));
    // addParallel TODO add rider angle moved command reverse

  }
}
