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
import frc.robot.subsystems.Rider;

public class Rocket1Command extends Command {
  private Intake intake = Intake.getInstance();
  private Rider rider = Rider.getInstance();
  private Elevator elevator = Elevator.getInstance();
  private int stage = 0;

  private Command intakeCommand, riderCommand, elevatorCommand , elevatorCommandPIDUp , intakeCommand2;

  
  private boolean elevatorUpFinished(){
    return !elevatorCommandPIDUp.isRunning(); 
  }

  private boolean intakeFinished() {
    return !intakeCommand.isRunning();
  }

  private boolean riderFinished() {
    return !riderCommand.isRunning();
  }

  private boolean elevatorFinished() {
    return !elevatorCommand.isRunning();
  }


  public Rocket1Command() {

    intakeCommand = new AutomaticIntake(-250,-300,1);
    riderCommand = new RiderPID(0, 0.3, 15);
    elevatorCommandPIDUp = new ElevatorPID(950,0);
    elevatorCommand = new ElevatorPID(1688, 0.2);
  }
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    stage=0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    switch (stage) {
      case 0:
        if (Intake.getInstance().getEncoder() > -200) {
          intakeCommand.start();
        }
        stage++;
        break;
      case 1:
        elevatorCommandPIDUp.start();
        stage++;
        break;
      case 2:
        if (elevatorUpFinished()) {
          riderCommand.start();
          stage++;
        }
        break;
      case 3:
        if (rider.getEncoder() < 100) {
          elevatorCommand.start();
          stage++;
        }
        break;
        case 4:
        if(elevator.getElevatorEncoder() > 100){
          intakeCommand.start();
        }
        break;
      }
    }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return elevatorFinished() && riderFinished() && intakeFinished() && stage == 4;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    
    intakeCommand.cancel();
    intake.intakeAngleControl(0);
    elevator.enablePID(false);
    OI.OPERATOR_STICK.setRumble(RumbleType.kLeftRumble, 1);
    OI.OPERATOR_STICK.setRumble(RumbleType.kRightRumble, 1);
    Timer.delay(0.5);
    OI.OPERATOR_STICK.setRumble(RumbleType.kLeftRumble, 0);
    OI.OPERATOR_STICK.setRumble(RumbleType.kRightRumble, 0);
   
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    intakeCommand.cancel();
    intake.intakeAngleControl(0);
    elevator.enablePID(false);
    
  }
}
