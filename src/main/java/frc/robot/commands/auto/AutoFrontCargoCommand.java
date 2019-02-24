/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.elevator.ElevatorPID;
import frc.robot.commands.intake.IntakePID;
import frc.robot.commands.rider.RiderPID;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OperatorControl;
import frc.robot.subsystems.Rider;

public class AutoFrontCargoCommand extends Command {
  Intake intake = Intake.getInstance();
  Rider rider = Rider.getInstance();
  Elevator elevator = Elevator.getInstance();

  private boolean isConditinalHappnds = true;

  private Command intakeCommand, riderCommand, elevatorCommand, intakeClosedCommand;

  public AutoFrontCargoCommand() {
    requires(OperatorControl.getInstance());

    elevatorCommand = new ElevatorPID(0, 0.1);
    intakeCommand = new IntakePID(-635, 0.1);
    riderCommand = new RiderPID(1185, 0.1, 15);
    intakeClosedCommand = new IntakePID(0, 0.1);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (isConditinalHappnds) {
      elevatorCommand.start();
      intakeCommand.start();
      if (elevator.getElevatorEncoder() < 1000 && intake.getEncoder() < -500) {
        riderCommand.start();
      }
      if (rider.getEncoder() > 900) {
        intakeClosedCommand.start();
      }
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return riderCommand.isCompleted() && intakeClosedCommand.isCompleted() && elevatorCommand.isCompleted();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    new RumbleJoystick(500);
    intake.enablePID(false);
    elevator.enablePID(false);
    rider.enablePID(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    intake.enablePID(false);
    elevator.enablePID(false);
    rider.enablePID(false);
  }
}
