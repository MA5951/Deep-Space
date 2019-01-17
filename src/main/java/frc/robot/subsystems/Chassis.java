/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.TankDrive;

/**
 * Add your docs here.
 */
public class Chassis extends Subsystem {

  private static Chassis c_Instance;
  
  private WPI_TalonSRX leftMotorOne;
  private WPI_TalonSRX leftMotorTwo;
  private WPI_TalonSRX leftMotorThree;
  private WPI_TalonSRX rightMotorOne;
  private WPI_TalonSRX rightMotorTwo;
  private WPI_TalonSRX rightMotorThree;

  private Encoder encoderRight;
  private Encoder encoderLeft;
  
  PIDController leftChassisPID;
  PIDController rightChassisPID;

  private double KP_CHASSIS = 0.0;
  private double KI_CHASSIS = 0.0;
  private double KD_CHASSIS = 0.0;
  private double tolerance = 0.5;


  public Chassis()
  {
    leftMotorOne = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_ONE);
    leftMotorTwo = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_TWO);
    leftMotorThree = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_THREE);
 
    rightMotorOne = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_ONE);
    rightMotorTwo = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_TWO);
    rightMotorThree = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_THREE);

    encoderLeft = new Encoder(RobotMap.ENCODER_LEFT_A,RobotMap.ENCODER_LEFT_B,false,EncodingType.k4X);
    encoderRight = new Encoder(RobotMap.ENCODER_RIGHT_A,RobotMap.ENCODER_RIGHT_B,false,EncodingType.k4X);

   
    rightMotorTwo.set(ControlMode.Follower,rightMotorOne.getDeviceID());
    rightMotorThree.set(ControlMode.Follower,rightMotorOne.getDeviceID());

    
    leftMotorTwo.set(ControlMode.Follower, leftMotorOne.getDeviceID());
    leftMotorThree.set(ControlMode.Follower, leftMotorOne.getDeviceID());

    leftChassisPID = new PIDController(KP_CHASSIS, KI_CHASSIS, KD_CHASSIS, encoderLeft, leftMotorOne);
    rightChassisPID = new PIDController(KP_CHASSIS, KI_CHASSIS, KD_CHASSIS, encoderLeft, rightMotorOne);

    rightChassisPID.setAbsoluteTolerance(tolerance);
    leftChassisPID.setAbsoluteTolerance(tolerance);

    rightChassisPID.enable();
    leftChassisPID.enable();

  }

  

  public void driveWestCoast(double speedLeft,double speedRight)
  {
    rightMotorOne.set(speedRight);
    leftMotorOne.set(speedLeft);

  }

  public boolean isRightOnTarget()
  {
    return rightChassisPID.onTarget();
  }

  public boolean isLeftOnTarget()
  {
    return  leftChassisPID.onTarget();
  }
 

  public void setPoint(double setPoint)
  {
    rightChassisPID.setSetpoint(setPoint);
    leftChassisPID.setSetpoint(setPoint);
  }


  public void enableChassisPID(boolean enable){
    if(enable){
      rightChassisPID.enable();
      leftChassisPID.enable();
    }
    else
      rightChassisPID.disable();
      leftChassisPID.disable();
   }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new TankDrive());

  }

  public static Chassis getInstance()
  {
    if(c_Instance==null)
    {
      c_Instance = new Chassis();
      return c_Instance;
    }
    return c_Instance;
  }
}
