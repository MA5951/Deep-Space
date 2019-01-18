/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;


public class Intake extends Subsystem {
public static Intake  instance;



public static Intake getInstance() {

if (instance == null) 
  instance = new Intake();

return instance;


}
private WPI_TalonSRX intake_speed;
private WPI_TalonSRX intake_position;
 



public void intake_speed() {
  intake_speed.set(0.5);
  
}

public void intake_positionUp() {
  intake_position.set(0.5);
}
public void intake_positionDown() {
  intake_position.set(-0.5);
}

  @Override
  public void initDefaultCommand() {


  }
}
