/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.commands.elevator.ElevatorPID;
import frc.robot.commands.intake.IntakePID;
import frc.robot.commands.rider.RiderPID;
import frc.robot.subsystems.Intake;


public class ReturnToDefault extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ReturnToDefault() {
    addSequential(
      new ConditionalCommand( new IntakePID(-635, 0.1), null){
      @Override
      protected boolean condition() {
        return Intake.getInstance().getEncoder() > -500;
      }
    });
    addSequential(new ElevatorPID(0, 0.1));
    addSequential(new RiderPID(0, 0.3, 15));
    addSequential(new IntakePID(0, 0));
    addSequential(new RumbleJoystick(500));
  }
}