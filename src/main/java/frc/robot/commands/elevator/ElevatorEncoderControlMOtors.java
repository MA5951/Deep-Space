/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorEncoderControlMOtors extends Command {
  Elevator elevator = Elevator.getInstance();
  double speed;
  double maxDistance;
  double minDistance;
  public ElevatorEncoderControlMOtors( double maxDistance, double minDistance,double speed) {
    this.speed = speed;
    this.maxDistance = maxDistance;
    this.minDistance = minDistance;
       requires(elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    elevator.controlSpeed(speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return elevator.isEncoderInDistanceRangeElevator(maxDistance, minDistance);
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

