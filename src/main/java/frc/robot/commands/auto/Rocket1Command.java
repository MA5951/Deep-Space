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
import frc.robot.subsystems.Rider;

public class Rocket1Command extends Command {
  Intake intake = Intake.getInstance();
  Rider rider = Rider.getInstance();
  Elevator elevator = Elevator.getInstance();

  private boolean isConditinalHappnds = false;

  private Command intakeCommand, riderCommand, elevatorCommand , elevatorCommandPIDUp;

  public Rocket1Command() {
    intakeCommand = new AutomaticIntake(-250,-300,1);
    riderCommand = new RiderPID(0, 0.3, 15);
    elevatorCommandPIDUp = new ElevatorPID(1447,0);
    elevatorCommand = new ElevatorPID(2495, 0.2);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(intake.getEncoder() > -200){
      new IntakePID(-600, 0.1);
      if (intake.getEncoder() <= (-200 - intake.TOLERANCE) || intake.getEncoder() >= (-200 + intake.TOLERANCE)) {
        isConditinalHappnds = true;
      }
    } else {
      isConditinalHappnds = true;
    }
    if (isConditinalHappnds) {
      elevatorCommandPIDUp.start();
      riderCommand.start();
    }
    if (rider.getEncoder() < 100) {
      elevatorCommand.start();
    }
    if (elevator.getElevatorEncoder() > 2000 && rider.getEncoder() < 500) {
      intakeCommand.start();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !intakeCommand.isRunning() && !elevatorCommand.isRunning() && !elevatorCommandPIDUp.isRunning()
        && !riderCommand.isRunning();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    new RumbleJoystick(500);
    intake.intakeAngleControl(0);
    elevator.enablePID(false);
    rider.enablePID(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    intake.intakeAngleControl(0);
    elevator.enablePID(false);
    rider.enablePID(false);
  }
}
