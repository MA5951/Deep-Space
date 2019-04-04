/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.commands.elevator.ElevatorPID;
import frc.robot.commands.intake.IntakePID;
import frc.robot.commands.rider.RiderPID;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OperatorControl;
import frc.robot.subsystems.Rider;

public class ReturnToDefaultCommand extends Command {
  private Intake intake = Intake.getInstance();
  private Rider rider = Rider.getInstance();
  private Elevator elevator = Elevator.getInstance();
  private int stage = 0;
  private long delayTime;

  private Command intakeCommand, riderCommand, elevatorCommand, moveIntakeCommand;

  private boolean intakeFinished() {
    return !intakeCommand.isRunning();
  }

  private boolean riderFinished() {
    return !riderCommand.isRunning();
  }

  private boolean elevatorFinished() {
    return !elevatorCommand.isRunning();
  }

  public ReturnToDefaultCommand() {
    requires(OperatorControl.getInstance());
    intakeCommand = new IntakePID(0, 0);
    moveIntakeCommand = new IntakePID(-635, 0.1);
    riderCommand = new RiderPID(10, 0.3, 15);
    elevatorCommand = new ElevatorPID(100, 0.1);
  }

  @Override
  protected void initialize() {
    delayTime =0;
    stage = 0;
    
  }

  @Override
  protected void execute() {
  
    switch (stage) {
    case 0:
      if (Intake.getInstance().getEncoder() > -500) {
        moveIntakeCommand.start();
      }
      stage++;
      break;
    case 1:
      if (!moveIntakeCommand.isRunning()) {
        stage++;
      }
      break;
    case 2:
      elevatorCommand.start();
      stage++;
      break;
    case 3:
      if (elevator.getElevatorEncoder() < 2000) {
        riderCommand.start();
        stage++;
      }
      break;
    case 4:
      if (rider.getEncoder() < 450) {
        intakeCommand.start();
        stage++;
      }
      break;
      case 5:
      if(intakeFinished()){
        OI.OPERATOR_STICK.setRumble(RumbleType.kLeftRumble, 1);
        OI.OPERATOR_STICK.setRumble(RumbleType.kRightRumble, 1);
        delayTime = System.currentTimeMillis();
        stage++;
      }
      break;
    case 6:
      if (System.currentTimeMillis() - delayTime > 500) {
        OI.OPERATOR_STICK.setRumble(RumbleType.kLeftRumble, 0);
        OI.OPERATOR_STICK.setRumble(RumbleType.kRightRumble, 0);
      }
        stage++;
    
      break;
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;   
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {

    intake.enablePID(false);
    

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    intake.enablePID(false);

    

  }
}
