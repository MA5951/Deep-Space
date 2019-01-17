/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;


public class Elevator extends Subsystem {
  private TalonSRX elevatorTalon;
  private DigitalInput elevatorLimitSwitchUpLeft;
 private DigitalInput elevatorLimitSwitchDownRigth;
 private DigitalInput elevatorLimitSwitchDownLeft;
 private DigitalInput elevatorLimitSwitchUpRigth;
 private Encoder elevatorEncoder;
 private static Elevator elevatorSubsystem;

private Elevator() {
  elevatorLimitSwitchUpLeft=new DigitalInput(RobotMap.LIMIT_SWICH_UP_LEFT);
  elevatorLimitSwitchUpRigth=new DigitalInput(RobotMap.LIMIT_SWICH_UP_RIGHT);
  elevatorLimitSwitchDownLeft=new DigitalInput(RobotMap.LIMIT_SWICH_DONW_LEFT);
  elevatorLimitSwitchDownRigth=new DigitalInput(RobotMap.LIMIT_SWICH_DONW_RIGHT);
  elevatorTalon=new TalonSRX(RobotMap.TALON_ELEVETOR);
  elevatorEncoder= new Encoder(RobotMap.ENCODER_ELEVETOR_B, RobotMap.ENCODER_ELEVETOR_A , false, EncodingType.k4X); 
  elevatorEncoder.setDistancePerPulse(1);
}
/**
* elevatortalonControlSpeed control the speed of the talon 
*/
public void elevatortalonControlSpeed(double speed){
  elevatorTalon.set(ControlMode.PercentOutput, speed);
}
/**
* elevatorLimitSwitchDown get the value of the down LimitSwitch's
*/
public boolean elevatorLimitSwitchDown(){
return elevatorLimitSwitchDownLeft.get() && elevatorLimitSwitchDownRigth.get();
}
/**
* elevatorLimitSwitchUp get the value of the up LimitSwitch's
*/
public boolean elevatorLimitSwitchUp(){
  return elevatorLimitSwitchUpLeft.get() && elevatorLimitSwitchUpRigth.get();
}
/**
* elevatorEncoderGetDistance checker if the value of the encoder is in thr Range
*/
public boolean elevatorEncoderGetDistance(double maxDistance, Double minDistance ){
  return  elevatorEncoder.getDistance()<maxDistance && elevatorEncoder.getDistance()> minDistance;
}
/**
* resetElevatorEncoder reset the encoder 
*/
public void resetElevatorEncoder(){
  elevatorEncoder.reset();
}
/**
* singalton 
*/
public static Elevator getInstance() {
  if (elevatorSubsystem == null) {
    elevatorSubsystem = new Elevator();
  }
  return elevatorSubsystem;
}
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
