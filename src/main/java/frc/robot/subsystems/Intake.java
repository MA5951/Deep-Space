/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  private WPI_TalonSRX dodoIntake;
  private WPI_TalonSRX dodoUpAndDonwLeft;
  private WPI_TalonSRX dodoUpAndDonwRight;
  private Encoder encoderDodo;
  private PIDController dodoPIDController;
  private Relay dodoSonoloidRigth;
  private Relay dodoSonoloidLeft;
  private static Intake dodoSubsystem;
  DigitalInput limitSWichUp;
  DigitalInput limitSWichDown;

  private Intake(){
    limitSWichUp=new DigitalInput(RobotMap.LINIT_SWHICH_UP);
    limitSWichDown=new DigitalInput(RobotMap.LINIT_SWHICH_DOWN);
    dodoSonoloidRigth=new Relay(RobotMap.RELAY_SOLONID_RIGHT);
    dodoSonoloidLeft=new Relay(RobotMap.RELAY_SOLONID_LEFT);
    dodoIntake=new WPI_TalonSRX(RobotMap.INTAKE_MOTORS_WILDES);
    dodoUpAndDonwRight=new WPI_TalonSRX(RobotMap.INTAKE_MOTORS_ANGLE_RIGHT);
    dodoUpAndDonwLeft=new WPI_TalonSRX(RobotMap.INTAKE_MOTORS_ANGLE_LEFT);
    dodoUpAndDonwRight.set(ControlMode.Follower, dodoUpAndDonwLeft.getDeviceID());
    encoderDodo=new Encoder(RobotMap.ENCODER_B_INTAKE, RobotMap.ENCODER_A_INTAKE,false,EncodingType.k4X);
    encoderDodo.setDistancePerPulse(1);
    encoderDodo.setPIDSourceType(PIDSourceType.kDisplacement);
    dodoPIDController=new PIDController(1, 1, 1,encoderDodo,dodoUpAndDonwLeft);
  }
  public boolean isLImitSwhichOnUp(){
return limitSWichUp.get();
  }
  public boolean isLImitSwhichOnDown(){
    return limitSWichDown.get();
  }
public void enablePID(){
  dodoPIDController.enable();
}
public void disablePID(){
  dodoPIDController.disable();
}
public void setSetpointPID(double setSetpoint){
  dodoPIDController.setSetpoint(setSetpoint);
}
public void tolerancePID(double Tolerance){
  dodoPIDController.setAbsoluteTolerance(Tolerance);
}
public boolean isOnTargetPID(){
  return dodoPIDController.onTarget();
}

public void dodoIntakeControl(double speed){
  dodoIntake.set(ControlMode.PercentOutput, speed);
}
public void dodoUpAndDonwLeftControl(double speedUpAndDown){
  dodoUpAndDonwLeft.set(ControlMode.PercentOutput, speedUpAndDown);
}
public void RelayControlFowerd(){
    dodoSonoloidRigth.set(Relay.Value.kForward);
    dodoSonoloidLeft.set(Relay.Value.kForward);
}
public void RelayControlRevers(){
    dodoSonoloidRigth.set(Relay.Value.kReverse);
    dodoSonoloidLeft.set(Relay.Value.kReverse);
}
public void resetEncoder(){
  encoderDodo.reset();
}
public static Intake getInstance() {
if(dodoSubsystem ==null){
  dodoSubsystem=new Intake();
}
return dodoSubsystem;
}
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
