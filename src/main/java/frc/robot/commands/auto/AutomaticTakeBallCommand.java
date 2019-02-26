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
  private Intake intake = Intake.getInstance();
  private Rider rider = Rider.getInstance();
  private Elevator elevator = Elevator.getInstance();

  private int stage = 0;

  private Command intakeCommand, riderCommand, elevatorCommand, intakeCommandIntake, riderCommandIntake;

  private boolean intakeFinished() {
    return !intakeCommand.isRunning();
  }

  private boolean riderFinished() {
    return !riderCommand.isRunning();
  }

  private boolean elevatorFinished() {
    return !elevatorCommand.isRunning();
  }

  public AutomaticTakeBallCommand() {
    requires(OperatorControl.getInstance());

    intakeCommand = new IntakePID(-800, 0.1);
    riderCommand = new RiderPID(600, 0.1, 15);
    elevatorCommand = new ElevatorPID(2900, 0.2);
    intakeCommandIntake = new IntakeMoveBall(-1.0);
    riderCommandIntake = new RiderIntake();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    stage = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println(stage);
    switch (stage) {
    case 0:
      intakeCommand.start();
      if (intake.getEncoder() <= -500) {
        stage++;
      }
      break;
    case 1:
      riderCommand.start();
      if (rider.getEncoder() >= 250) {
        stage++;
      }
      break;
    case 2:

      elevatorCommand.start();
      stage++;
      break;
    case 3:
      if (elevatorFinished()) {
        riderCommand.cancel();
        intakeCommand.cancel();
        elevatorCommand.cancel();
        stage++;
      }
      break;
    case 4:
      if (!rider.getBallLimitswitch()) {
        riderCommandIntake.start();
        intakeCommandIntake.start();
        stage++;
      }
      break;
    case 5:
      if (rider.getBallLimitswitch()) {
        stage++;
      }
      break;
    case 6:
      riderCommandIntake.cancel();
      intakeCommandIntake.cancel();
      stage++;
      break;
    }
  }

  @Override
  protected boolean isFinished() {
    return stage == 7;
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
    OI.OPERATOR_STICK.setRumble(RumbleType.kLeftRumble, 0);
    OI.OPERATOR_STICK.setRumble(RumbleType.kRightRumble, 0);
    riderCommandIntake.cancel();
    intakeCommandIntake.cancel();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    intake.enablePID(false);
    elevator.enablePID(false);
    rider.enablePID(false);
    riderCommandIntake.cancel();
    intakeCommandIntake.cancel();
  }
}
