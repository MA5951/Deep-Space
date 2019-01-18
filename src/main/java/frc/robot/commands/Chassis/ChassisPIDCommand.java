/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Chassis;

public class ChassisPIDCommand extends Command {

  private Chassis chassis;
  private double distance;

  public ChassisPIDCommand(double distance) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.distance = distance;
    chassis = Chassis.getInstance();
    requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    chassis.enableChassisPID(true);
    chassis.setPoint(distance);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return chassis.isLeftOnTarget() && chassis.isRightOnTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    chassis.enableChassisPID(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    
  }
}
