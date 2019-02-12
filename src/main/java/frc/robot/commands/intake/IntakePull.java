/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Intake;

// USE COMMAND GROUP INSTEAD

public class IntakePull extends Command {

 
  private Intake intake = Intake.getInstance();



  /**
   * Creates new {IntakePull} command.
   * 
   * @param speed the given power of {intakeControl}.
   */
  public IntakePull() {
   
    requires(intake);
    //requires(rider);
  }

  @Override
  protected void initialize() {

  }

  /**
   * Give power to the {intakeControl}.
   */
  @Override
  protected void execute() {
    intake.intakeBallControl(-1);
  }

  /**
   * Check whether limit switch is pressed.
   */
  @Override
  protected boolean isFinished() {
    return false; 
  }

  /**
   * 
   * 
   * Disables the {intakeControl} function if {isFinished} function return true.
   */
  @Override
  protected void end() {
    intake.intakeBallControl(0);
  }

  /**
   * Diables the {intakeControl} function if {end} function was interrupted.
   */
  @Override
  protected void interrupted() {
    intake.intakeBallControl(0);
  }
}
