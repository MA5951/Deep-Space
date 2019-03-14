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

public class AutoFrontCargoCommand extends Command {
  private Intake intake = Intake.getInstance();
  private Rider rider = Rider.getInstance();
  private Elevator elevator = Elevator.getInstance();
  private int stage = 0;
  private long delayTime;

  private Command intakeCommand, riderCommand, elevatorCommand, intakeClosedCommand;

  private boolean intakeFinishedCLosed() {
    return !intakeClosedCommand.isRunning();
  }

  public AutoFrontCargoCommand() {
    requires(OperatorControl.getInstance());

    elevatorCommand = new ElevatorPID(50, 0.1);
    intakeCommand = new IntakePID(-800, 0.1);
    riderCommand = new RiderPID(1220, 0.1, 15);
    intakeClosedCommand = new IntakePID(0, 0.1);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    delayTime= 0 ;
    stage = 0;
    System.out.println("[" + Timer.getMatchTime() + "]" + " (AutoFrontCargoCommand) - " + "Command initialized. ");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println(stage);
    switch (stage) {
    case 0:
      if (elevator.getElevatorEncoder() > 2500 && rider.getEncoder() > 300) {
        elevatorCommand.start();
        stage++;
      } else {
        stage++;
      }
      break;
    case 1:
      elevatorCommand.start();
      intakeCommand.start();
      if (elevator.getElevatorEncoder() < 1000 && intake.getEncoder() < -500) {
        stage++;
      }
      break;
    case 2:
      riderCommand.start();
      if (elevator.getElevatorEncoder() < 100 && rider.getEncoder() > 1000) {
        stage++;
      }
      break;
    case 3:
      intakeClosedCommand.start();
      stage++;
      break;
      case 4:
      if(intakeFinishedCLosed()){
        OI.OPERATOR_STICK.setRumble(RumbleType.kLeftRumble, 1);
        OI.OPERATOR_STICK.setRumble(RumbleType.kRightRumble, 1);
        delayTime = System.currentTimeMillis();
        stage++;
      }
      break;
    case 5:
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
    System.out.println("command interrupted");


  }
}
