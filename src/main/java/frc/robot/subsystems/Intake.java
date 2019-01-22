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
  
  //motors
  private WPI_TalonSRX ballDeskeetTalon;
  private WPI_TalonSRX intakeMovementLeft;
  private WPI_TalonSRX intakeMovementRight;

  //encoder
  private Encoder encoderIntake;

  //PID
  private PIDController IntakePIDController;

  //Electric Pistons
  private Relay intakePistonRight;
  private Relay intakePistonLeft;


  private static Intake intakeSubsystem;

  //sensors
  private DigitalInput limitSWichUp;
  private DigitalInput limitSWichDown;


  private Intake(){

    limitSWichUp = new DigitalInput(RobotMap.LINIT_SWHICH_UP);
    limitSWichDown = new DigitalInput(RobotMap.LINIT_SWHICH_DOWN);

    intakePistonRight = new Relay(RobotMap.RELAY_SOLONID_RIGHT);
    intakePistonLeft = new Relay(RobotMap.RELAY_SOLONID_LEFT);
    
    ballDeskeetTalon = new WPI_TalonSRX(RobotMap.INTAKE_MOTORS_WILDES);
    
    intakeMovementRight = new WPI_TalonSRX(RobotMap.INTAKE_MOTORS_ANGLE_RIGHT);
    intakeMovementLeft = new WPI_TalonSRX(RobotMap.INTAKE_MOTORS_ANGLE_LEFT);
    intakeMovementRight.set(ControlMode.Follower, intakeMovementLeft.getDeviceID());
    intakeMovementLeft.setInverted(true);
    
    encoderIntake = new Encoder(RobotMap.ENCODER_B_INTAKE, RobotMap.ENCODER_A_INTAKE,false,EncodingType.k4X);
    encoderIntake.setDistancePerPulse(1);
    encoderIntake.setPIDSourceType(PIDSourceType.kDisplacement);
    
    IntakePIDController=new PIDController(1, 1, 1,encoderIntake,intakeMovementLeft);
  }

  public boolean isLImitSwhichOnUp(){
    return limitSWichUp.get();
  }

  public boolean isLImitSwhichOnDown(){
    return limitSWichDown.get();
  }

  public void enablePID(){
    IntakePIDController.enable();
}
  public void disablePID(){
    IntakePIDController.disable();
}

  public void setSetpointPID(double setSetpoint){
    IntakePIDController.setSetpoint(setSetpoint);
}

  public void tolerancePID(double Tolerance){
   IntakePIDController.setAbsoluteTolerance(Tolerance);
}
  public boolean isOnTargetPID(){
    return IntakePIDController.onTarget();
}

  public void intakeControl(double speed){
    ballDeskeetTalon.set(ControlMode.PercentOutput, speed);
}

  public void intakeMovmentControl(double speedUpAndDown){
    intakeMovementLeft.set(ControlMode.PercentOutput, speedUpAndDown);
}

  public void RelayControlFowerd(){
    intakePistonRight.set(Relay.Value.kForward);
    intakePistonLeft.set(Relay.Value.kForward);
}

  public void RelayControlRevers(){
    intakePistonRight.set(Relay.Value.kReverse);
    intakePistonLeft.set(Relay.Value.kReverse);
} 
  public void resetEncoder(){
    encoderIntake.reset();
}

  public static Intake getInstance() {
    if(intakeSubsystem ==null){
      intakeSubsystem=new Intake();
    }
    return intakeSubsystem;
}
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
