/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.chassis.TankDrive;

/**
 * Add your docs here.
 */
public class Chassis extends Subsystem {

  private static Chassis c_Instance;
  
  private WPI_TalonSRX leftFrontMotor;
  private WPI_TalonSRX leftMiddleMotor;
  private WPI_TalonSRX leftRearMotor;
  private WPI_TalonSRX rightFrontMotor;
  private WPI_TalonSRX rightMiddleMotor;
  private WPI_TalonSRX rightRearMotor;

  private Encoder encoderRight;
  private Encoder encoderLeft;
  
  private PIDController leftChassisPID;
  private PIDController rightChassisPID;

  //TODO
  public static final double KP_CHASSIS = 0.0;
  public static final double KI_CHASSIS = 0.0;
  public static final double KD_CHASSIS = 0.0;
  public static final double TOLERANCE = 0.5;
  public static final double DISTANCE_PER_PULSE = 0.01;

  public Chassis()
  {
    leftFrontMotor = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_ONE);
    leftMiddleMotor = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_TWO);
    leftRearMotor = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_THREE);
 
    rightFrontMotor = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_ONE);
    rightMiddleMotor = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_TWO);
    rightRearMotor = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_THREE);

    encoderLeft = new Encoder(RobotMap.ENCODER_LEFT_A,RobotMap.ENCODER_LEFT_B,false,EncodingType.k4X);
    encoderRight = new Encoder(RobotMap.ENCODER_RIGHT_A,RobotMap.ENCODER_RIGHT_B,false,EncodingType.k4X);

    encoderLeft.reset();
    encoderRight.reset();

    encoderLeft.setDistancePerPulse(DISTANCE_PER_PULSE);
    encoderRight.setDistancePerPulse(DISTANCE_PER_PULSE);

    encoderLeft.setPIDSourceType(PIDSourceType.kDisplacement);
    encoderRight.setPIDSourceType(PIDSourceType.kDisplacement);

    rightMiddleMotor.set(ControlMode.Follower,rightFrontMotor.getDeviceID());
    rightRearMotor.set(ControlMode.Follower,rightFrontMotor.getDeviceID());

    
    leftMiddleMotor.set(ControlMode.Follower, leftFrontMotor.getDeviceID());
    leftRearMotor.set(ControlMode.Follower, leftFrontMotor.getDeviceID());

    leftChassisPID = new PIDController(KP_CHASSIS, KI_CHASSIS, KD_CHASSIS, encoderLeft, leftFrontMotor);
    rightChassisPID = new PIDController(KP_CHASSIS, KI_CHASSIS, KD_CHASSIS, encoderRight, rightFrontMotor);

    rightChassisPID.setAbsoluteTolerance(TOLERANCE);
    leftChassisPID.setAbsoluteTolerance(TOLERANCE);

    rightChassisPID.enable();
    leftChassisPID.enable();

  }

  

  public void driveWestCoast(double speedLeft,double speedRight)  {
    
    rightFrontMotor.set(speedRight);
    leftFrontMotor.set(speedLeft);
  }

  public boolean isRightOnTarget()  {

    return rightChassisPID.onTarget();
  }

  public boolean isLeftOnTarget() {

    return  leftChassisPID.onTarget(); 
  }
 

  public void setPoint(double setPoint) {

    rightChassisPID.setSetpoint(setPoint);
    leftChassisPID.setSetpoint(setPoint);
  }


  public void enableChassisPID(boolean enable){

    if(enable){
      rightChassisPID.enable();
      leftChassisPID.enable();
    }
    else{
      rightChassisPID.disable();
      leftChassisPID.disable();
    }
   }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new TankDrive());
  }


  public static Chassis getInstance()
  {
    if(c_Instance==null)
      
      c_Instance = new Chassis();
  
    return c_Instance;
  }
}
