/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Intake;

public class IntakeMovement extends Command {

  private Intake intake = Intake.getInstance();
  Double speed;

  /**
   * Creates new {IntakeMovement} command.
   * 
   * @param speedUpAndDown the given power of {intakeMovmentControl} motor.
   */
  public IntakeMovement(Double speed) {
    this.speed = speed;

    requires(intake);
  }

  /**
   * Give power to the {intakeMovmentControl} motor.
   */
  @Override
  protected void initialize() {

  }

  @Override
  protected void execute() {
    intake.intakeAngleControl(speed);
    if (intake.isIntakeLimitswitchClosed()){
      intake.resetEncoder();
    }
      if(intake.getEncoder() < -1400 && speed < 0 && Timer.getMatchTime() < 150 && OI.RIGHT_DRIVER_STICK.getRawAxis(3) > 0.5 ){
        intake.intakeAngleControl(0);
      }
}  
     

  /**
   * Check whether limit switch is pressed.
   */
  @Override
  protected boolean isFinished() {
    return speed > 0 && intake.isIntakeLimitswitchClosed();
    
  }

  /**
   * Disable the {intakeMovmentControl} function if {isFinished} function return
   * true.
   */
  @Override
  protected void end() {
    intake.intakeAngleControl(0);
  }

  /**
   * Disable the {intakeMovmentControl} function if {end} function was
   * interrupted.
   */
  @Override
  protected void interrupted() {
    intake.intakeAngleControl(0);
  }
}
