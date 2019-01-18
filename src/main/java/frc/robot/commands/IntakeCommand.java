/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Joystick;
public class IntakeCommand extends Command {
  
  
	private Intake intake = Intake.getInstance();
  

  public IntakeCommand() {

    requires(intake);

  }

  @Override
  protected void initialize() {

    

  }

 
  @Override
  protected void execute() {
    
    Intake.getInstance();



  }
// Make this return true when this Command no longer needs to run execute()
  
  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    intake.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    intake.stop();
  }
}
