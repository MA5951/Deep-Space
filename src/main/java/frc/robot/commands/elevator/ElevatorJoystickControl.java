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
  Elevator elevator; // TODO Encapsulation please
   double speed; // TODO unnecessary member 
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
    speed=OI.OPERATOR_STICK.getRawAxis(1);
    elevator.controlSpeed(speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() { // TODO isfinished is supposed to be false in default command, add safety inside execute and interrupted. 
    return elevator.isLimitSwitchUpPressed()&&speed>0||elevator.isLimitSwitchDownPressed()&&speed<0;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    elevator.controlSpeed(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    elevator.controlSpeed(0);
  }
}
