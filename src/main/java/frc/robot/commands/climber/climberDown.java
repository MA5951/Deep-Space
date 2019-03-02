/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Climber;

/**
 * Add your docs here.
 */
public class climberDown extends InstantCommand {
  Climber climber = Climber.getInstance();
  public static boolean yesOrNot = false;
  /**
   * Add your docs here.
   */
  public climberDown() {
    requires(climber);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    yesOrNot = !yesOrNot;
    if(yesOrNot == true){
      climber.kforwardeClamber();
    }
    if(yesOrNot == false){
     climber.kReverseClimber();
    }
    
  }

}
