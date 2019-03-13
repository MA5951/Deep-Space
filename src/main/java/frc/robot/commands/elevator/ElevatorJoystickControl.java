/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Elevator;

public class ElevatorJoystickControl extends Command {
  Elevator elevator;
  private boolean firstPID_Run = true;

  public ElevatorJoystickControl() {
    elevator = Elevator.getInstance();
    requires(elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (OI.OPERATOR_STICK.getRawAxis(1) < 0.1 && OI.OPERATOR_STICK.getRawAxis(1) > -0.1) {
      if (firstPID_Run) {
        elevator.setSetPoint(elevator.getElevatorEncoder());
        elevator.enablePID(true);
        firstPID_Run = false;
      }
    }
    if (OI.OPERATOR_STICK.getRawAxis(1) > 0.1 || OI.OPERATOR_STICK.getRawAxis(1) < -0.1) {
      if (elevator.isElevatorLimitswitchUpPressed() && OI.OPERATOR_STICK.getRawAxis(1) < 0) {
        elevator.setSetPoint(elevator.getElevatorEncoder());
        elevator.enablePID(true);
        firstPID_Run = false;
      } else {
        elevator.enablePID(false);
        firstPID_Run = true;
        elevator.controlSpeed(OI.OPERATOR_STICK.getRawAxis(1)*0.7);
      }
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    
  }
}
