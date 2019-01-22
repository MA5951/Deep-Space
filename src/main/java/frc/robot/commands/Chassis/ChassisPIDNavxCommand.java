/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Chassis;

public class ChassisPIDNavxCommand extends Command {

  private Chassis chassis;
  // TODO
  private double distance = 0;

  /**
   * create new distance (set point) parameter
   * 
   * @param distance the given distance (set point)
   */
  public ChassisPIDNavxCommand(double distance) {

    this.distance = distance;
    chassis = Chassis.getInstance();
    requires(chassis);
  }

  /**
   * Enable the PID and set the distance (set point)
   */
  protected void initialize() {
    chassis.enableChassisNavxPID(true);
    chassis.setSetPoint(distance);
  }

  /**
   * Return if the robot reached the desire distance (set point)
   */
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return chassis.isLeftNavxPIDOnTarget() && chassis.isRightOnTarget();
  }

   /**
   * Disable the PIDController
   */
  @Override
  protected void end() {
    chassis.enableChassisNavxPID(false);
  }

  /**
   * Disable the PIDController if end() was interrupted
   */
  @Override
  protected void interrupted() {
    chassis.enableChassisNavxPID(false);
  }
}
