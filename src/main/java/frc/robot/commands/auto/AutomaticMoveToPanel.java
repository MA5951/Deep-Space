/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Chassis;

public class AutomaticMoveToPanel extends Command {
  Chassis chassis = Chassis.getInstance();
  public AutomaticMoveToPanel() {
    // Use requires() here to declare subsystem dependencies
     requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double speed = SmartDashboard.getNumber("x", 0);
    double lPower = SmartDashboard.getNumber("inputPowerL", 0);
    double rPower = SmartDashboard.getNumber("inputPowerR", 0);
    double r = 0.5*Math.min(1, 1-Math.sin(Math.toRadians(speed))); //lPower;
    double l = 0.5*Math.min(1, 1-Math.sin(Math.toRadians(-speed))); //rPower;
    chassis.driveWestCoast(-l, -r);
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
