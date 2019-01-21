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
 * The Chassis subsystem
 */
public class Chassis extends Subsystem {

  private static Chassis c_Instance;
  
  private WPI_TalonSRX leftFrontMotor;
  private WPI_TalonSRX leftRearMotor;

  private WPI_TalonSRX rightFrontMotor;
  private WPI_TalonSRX rightRearMotor;


  private Encoder encoderRight;
  private Encoder encoderLeft;
  
  private PIDController leftChassisPID;
  private PIDController rightChassisPID;

  //TODO
  public static final double KP_CHASSIS = 0.0;
  public static final double KI_CHASSIS = 0.0;
  public static final double KD_CHASSIS = 0.0;
  private static final double TOLERANCE = 0.5;
  private static final double DISTANCE_PER_PULSE = 0.01;


  /**
   * Initializes all Chassis components
   */
  public Chassis()
  {
    leftFrontMotor = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_ONE);
    leftRearMotor = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_TWO);

 
    rightFrontMotor = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_ONE);
    rightRearMotor = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_TWO);
   

    encoderLeft = new Encoder(RobotMap.ENCODER_LEFT_A,RobotMap.ENCODER_LEFT_B,false,EncodingType.k4X);
    encoderRight = new Encoder(RobotMap.ENCODER_RIGHT_A, RobotMap.ENCODER_RIGHT_B, false, EncodingType.k4X);

    encoderLeft.reset();
    encoderRight.reset();

    encoderLeft.setDistancePerPulse(DISTANCE_PER_PULSE);
    encoderRight.setDistancePerPulse(DISTANCE_PER_PULSE);

    encoderLeft.setPIDSourceType(PIDSourceType.kDisplacement);
    encoderRight.setPIDSourceType(PIDSourceType.kDisplacement);

    rightFrontMotor.set(ControlMode.Follower,rightFrontMotor.getDeviceID());
    rightRearMotor.set(ControlMode.Follower,rightFrontMotor.getDeviceID());

    
    leftFrontMotor.set(ControlMode.Follower, leftFrontMotor.getDeviceID());
    leftRearMotor.set(ControlMode.Follower, leftFrontMotor.getDeviceID());

    leftChassisPID = new PIDController(KP_CHASSIS, KI_CHASSIS, KD_CHASSIS, encoderLeft, leftFrontMotor);
    rightChassisPID = new PIDController(KP_CHASSIS, KI_CHASSIS, KD_CHASSIS, encoderRight, rightFrontMotor);

    rightChassisPID.setAbsoluteTolerance(TOLERANCE);
    leftChassisPID.setAbsoluteTolerance(TOLERANCE);

    rightChassisPID.enable();
    leftChassisPID.enable();

  }

  
  /**
   * Give power to the motors
   * @param speedLeft Speed of the left side chassis
   * @param speedRight Speed of the right side chassis
   */
  public void driveWestCoast(double speedLeft,double speedRight)  {
    
    rightFrontMotor.set(speedRight);
    leftFrontMotor.set(speedLeft);
  }

  /**
   * Check whether right PIDController is on target
   * @return Indication if right controller is on target
   */
  public boolean isRightOnTarget()  {

    return rightChassisPID.onTarget();
  }

  /**
   * Check whether right PIDController is on target
   * @return Indication if right controler is on target
   */
  public boolean isLeftOnTarget() {

    return  leftChassisPID.onTarget(); 
  }
 

  /**
   * Set the PID destination (set point)
   * @param setPoint The given destination (set point)
   */
  public void setSetPoint(double setPoint) {

    rightChassisPID.setSetpoint(setPoint);
    leftChassisPID.setSetpoint(setPoint);
  }


  /**
   * Enable the PID if true and disable if false
   * @param enable Enable the function
   */
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


  /**
   * Makes sure that only one intance runs at a time
   * @return Return the instance
   */
  public static Chassis getInstance()
  {
    if(c_Instance==null)
      
      c_Instance = new Chassis();
  
    return c_Instance;
  }
}
