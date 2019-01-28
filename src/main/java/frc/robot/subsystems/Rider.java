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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

// TODO Redesign rider commands after redesigning subsystem. 
@Deprecated
public class Rider extends Subsystem { // TODO Add javadoc
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Rider r_Instance; 

  private WPI_TalonSRX angleMotor;
  private WPI_TalonSRX intakeMotor;

  private Encoder encoderAngle;

  private DigitalInput limitSwitchA; // TODO Change Sensors completely. 
  private DigitalInput limitSwitchB;  // Limit switch angle up, limit switch angle down, proximity sensor. 

  private PIDController angleController; // TODO angleEncoderPID

  public static final double KP_ANGLE = 0;
  public static final double KI_ANGLE = 0;
  public static final double KD_ANGLE = 0;

  public static final double DISTANCE_PER_PULSE = 1.0;

  /**
   * Initializes all Rider components
   */
  private Rider() {

    angleMotor = new WPI_TalonSRX(RobotMap.RIDER_ANGLE_MOTOR);
    intakeMotor = new WPI_TalonSRX(RobotMap.RIDER_INTAKE_MOTOR);

    encoderAngle = new Encoder(RobotMap.RIDER_ENCODER_A, RobotMap.RIDER_ENCODER_B, false, EncodingType.k4X);
    encoderAngle.setDistancePerPulse(DISTANCE_PER_PULSE);

    limitSwitchA = new DigitalInput(RobotMap.RIDER_ANGLE_LIMIT_SWITCH);
    limitSwitchB = new DigitalInput(RobotMap.RIDER_INTAKE_LIMIT_SWITCH);

    angleController = new PIDController(KP_ANGLE, KI_ANGLE, KD_ANGLE, encoderAngle, angleMotor);
    encoderAngle.setPIDSourceType(PIDSourceType.kDisplacement);
  }

  /**
   * Enables the angle PID
   */
  public void enablePID() { // TODO Change to one function with parameter. 
    angleController.enable();
  }

  /**
   * Disables the angle PID
   */
  public void disablePID() {
    angleController.disable();
  }

  /**
   * Set the desired angle for the Rider.
   * 
   * @param setPoint the destination.
   */
  public void setSetPoint(double setPoint) {
    angleController.setSetpoint(setPoint);
  }

  /**
   * Set the set point tolerance
   * 
   * @param tolerance The given tolerance
   */
  public void setPIDTolerance(double tolerance) { // TODO no need for function, define tolerance in constant inside subsystem
    angleController.setAbsoluteTolerance(tolerance);
  }

  /**
   * 
   * Return if the robot reached the desired destination
   * 
   * @return Indication if rider is on target
   */
  public boolean isOnTarget() {
    return angleController.onTarget();
  }

  /**
   * Control the intake motor power
   * 
   * @param speed The given power
   */
  public void setIntakeMotor(double speed) { // TODO controlIntakeMotor
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Control the angle motor power
   * 
   * @param angleSpeed The given power
   */
  public void setAngleMotor(double angleSpeed) { // TODO controlAngleMotor
    angleMotor.set(ControlMode.PercentOutput, angleSpeed);
  }

  public boolean isLimitSwitchAnglePressed() { // TODO Delete this function. Redesign with new sensors. 
    return limitSwitchA.get() || limitSwitchB.get();
  }

  public boolean isAngleInRange(double angle, double tolerance) {
    return encoderAngle.get() < angle + tolerance && encoderAngle.get() > angle - tolerance;
  }

  public int getCurrentAngle() {
    return encoderAngle.get();
  }

  /**
   * Resets the encoder
   */
  public void resetEncoder() {
    encoderAngle.reset();
  }

  /**
   * Makes sure that only one intance runs at a time
   * 
   * @return The instance
   */
  public static Rider getInstance() {
    if (r_Instance == null)
      r_Instance = new Rider();
    return r_Instance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here
  }
}
