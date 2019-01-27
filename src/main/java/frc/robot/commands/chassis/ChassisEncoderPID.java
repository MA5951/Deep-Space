/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Chassis;

@Deprecated
public class ChassisEncoderPID extends Command {

  private Chassis chassis;
  // TODO
  private double distance;

  /**
   * create new distance (set point) parameter
   * 
   * @param distance the given distance (set point)
   */
  public ChassisEncoderPID(double distance) {

    this.distance = distance;
    chassis = Chassis.getInstance();
    requires(chassis);
  }

  /**
   * Enable the PID and set the distance (set point)
   */
  @Override
  protected void initialize() {
    chassis.enableChassisEncoderPID(true);
    chassis.setSetPointEncoder(distance);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  /**
   * Return if the robot reached the desire distance (set point)
   */
  @Override
  protected boolean isFinished() {
    return chassis.isLeftEncoderPIDOnTarget() && chassis.isRightEncoderPIDOnTarget();
  }

  /**
   * Disable the PID
   */
  @Override
  protected void end() {
    chassis.enableChassisEncoderPID(false);
  }

  /**
   * Run the end function if it was interrupted
   */
  @Override
  protected void interrupted() {
    end();
  }
}
