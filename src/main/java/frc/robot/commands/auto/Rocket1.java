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

public class Rocket1 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Rocket1() {
    addSequential(
      new ConditionalCommand( new IntakePID(-600, 0.1), null){
      @Override
      protected boolean condition() {
        return Intake.getInstance().getEncoder() > -200;
      }
        });
    
  
    addSequential(new ElevatorPID(1447, 0));
    addSequential(new RiderPID(0, 1, 20));
    addSequential(new ElevatorPID(2495, 0.2));
    addSequential(new AutomaticIntake(-250,-300,1));
     addSequential(new RumbleJoystick(500));

  }
}
