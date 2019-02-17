/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;

public class RumbleJoystick extends Command {

  long startTime = -1;
  double time = -1;

  public RumbleJoystick(double time) {
    this.time = time;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    OI.OPERATOR_STICK.setRumble(RumbleType.kLeftRumble, 1);
    OI.OPERATOR_STICK.setRumble(RumbleType.kRightRumble, 1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return System.currentTimeMillis() - startTime >= time;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    OI.OPERATOR_STICK.setRumble(RumbleType.kLeftRumble, 0);
    OI.OPERATOR_STICK.setRumble(RumbleType.kRightRumble, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    OI.OPERATOR_STICK.setRumble(RumbleType.kLeftRumble, 0);
    OI.OPERATOR_STICK.setRumble(RumbleType.kRightRumble, 0);
  }
}
