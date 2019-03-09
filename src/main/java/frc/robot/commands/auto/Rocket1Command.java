/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.commands.elevator.ElevatorPID;
import frc.robot.subsystems.Intake;

public class Rocket1Command extends Command {
  private Intake intake = Intake.getInstance();
  private int stage = 0;
  private Command intakeCommand2, elevatorCommand2;

  private boolean intakeFinished2() {
    return !intakeCommand2.isRunning();
  }

  private boolean elevatorFinished() {
    return !intakeCommand2.isRunning();
  }

  public Rocket1Command() {
    intakeCommand2 = new AutomaticIntake(-250, -350, 0.7);
    elevatorCommand2 = new ElevatorPID(2300, 0.1);

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
      intakeCommand2.start();
      stage++;
      break;
    case 1:
      if (intake.getEncoder() < 150) {
        elevatorCommand2.start();
        stage++;
      }
      break;
    case 2:
      if (elevatorFinished() && intakeFinished2()) {
        OI.OPERATOR_STICK.setRumble(RumbleType.kLeftRumble, 1);
        OI.OPERATOR_STICK.setRumble(RumbleType.kRightRumble, 1);
        Timer.delay(0.5);
        OI.OPERATOR_STICK.setRumble(RumbleType.kLeftRumble, 0);
        OI.OPERATOR_STICK.setRumble(RumbleType.kRightRumble, 0);
        stage++;
      }
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
    intake.intakeAngleControl(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    intake.enablePID(false);
    intake.intakeAngleControl(0);

  }
}
