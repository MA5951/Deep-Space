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
import frc.robot.commands.intake.IntakeMoveBall;
import frc.robot.commands.intake.IntakePID;
import frc.robot.commands.rider.RiderIntake;
import frc.robot.commands.rider.RiderPID;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OperatorControl;
import frc.robot.subsystems.Rider;

public class AutomaticTakeBallCommand extends Command {
  
  Intake intake = Intake.getInstance();
  Rider rider = Rider.getInstance();
  Elevator elevator = Elevator.getInstance();

  private boolean isConditinalHappnds = false;

  private Command intakeCommand, riderCommand, elevatorCommand, intakeCommandIntake, riderCommandIntake;

  private boolean intakeFinished () {
    return intake.getEncoder() > -800 - intake.TOLERANCE 
    && intake.getEncoder() < -800 + intake.TOLERANCE;
  }

  private boolean riderFinished () {
    return rider.getEncoder() < 600 + rider.TOLERANCE 
    && rider.getEncoder() > 600 - rider.TOLERANCE;
  }

  private boolean elevatorFinished () {
    return elevator.getElevatorEncoder() > 3050 - elevator.TOLERANCE
    &&  elevator.getElevatorEncoder() < 3050 + elevator.TOLERANCE;
  }

  public AutomaticTakeBallCommand() {
    requires(OperatorControl.getInstance());

    intakeCommand = new IntakePID(-800, 0.1);
    riderCommand = new RiderPID(600, 0.1, 15);
    elevatorCommand = new ElevatorPID(3050, 0.2);
    intakeCommandIntake = new IntakeMoveBall(-1.0);
    riderCommandIntake = new RiderIntake();

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (rider.getEncoder() > -500) {
      new ElevatorPID(1270, 0.1);
      if (elevator.getElevatorEncoder() >= (1270 - elevator.TOLERANCE)
          || elevator.getElevatorEncoder() <= (1270 + elevator.TOLERANCE)) {
        isConditinalHappnds = true;
      }
    } else {
      isConditinalHappnds = true;
    }
    if (isConditinalHappnds) {
      intakeCommand.start();
      if (intake.getEncoder() <= -400) {
        riderCommand.start();
        
      }
      if (rider.getEncoder() >= 187) {
        elevatorCommand.start();
      }
    }  
    if (intakeFinished() && elevatorFinished() && riderFinished()) {
      intake.enablePID(false);
      elevator.enablePID(false);
      rider.enablePID(false);
      intakeCommandIntake.start();
      riderCommandIntake.start();
    }
  }
    
  

  @Override
  protected boolean isFinished() {
    return intakeFinished() && elevatorFinished() && riderFinished() && rider.getBallLimitswitch();
  }

  

  // Called once after isFinished returns true
  @Override
  protected void end() {
    intake.enablePID(false);
    elevator.enablePID(false);
    rider.enablePID(false);
    OI.OPERATOR_STICK.setRumble(RumbleType.kLeftRumble, 1);
    OI.OPERATOR_STICK.setRumble(RumbleType.kRightRumble, 1);
    Timer.delay(0.1);
    OI.OPERATOR_STICK.setRumble(RumbleType.kLeftRumble , 0);
    OI.OPERATOR_STICK.setRumble(RumbleType.kRightRumble ,0);
    intake.intakeBallControl(0);
    rider.controlIntakeMotor(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    intake.enablePID(false);
    elevator.enablePID(false);
    rider.enablePID(false);
    intake.intakeBallControl(0);
    rider.controlIntakeMotor(0);
  }
}
