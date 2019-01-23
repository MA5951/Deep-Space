/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Chassis;

public class ChassisPIDNavxCommand extends Command {

  private Chassis chassis;
  // TODO
  private double distance;
  private double angle;

  /**
   * Creates a new Navx PID command.
   * 
   * @param distance the given distance to drive.
   * @param angle    The given angle to maintain (set point)
   */
  public ChassisPIDNavxCommand(double distance, double angle) {
    this.distance = distance;
    this.angle = angle;
    chassis = Chassis.getInstance();
    requires(chassis);
  }

  /**
   * Creates a new Navx PID command. Default setpoint is 0 degrees.
   * 
   * @param distance the given distance to drive.
   */
  public ChassisPIDNavxCommand(double distance) {
    this(distance, 0);
  }

  /**
   * Enable the PID and set the distance (set point)
   */
  protected void initialize() {
    chassis.enableChassisNavxPID(true);
    chassis.setSetPointNavx(angle);
  }

  /**
   * Return if the robot reached the desire distance (set point)
   */
  @Override
  protected void execute() {
    chassis.driveSingleSide(0.5);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true; //TODO
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
