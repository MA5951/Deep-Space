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

public class Rider extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Rider instance;

  private WPI_TalonSRX angleMotor;
  private WPI_TalonSRX intakeMotor;
  private Encoder encoderAngle;
  private DigitalInput limitSwitchA;
  private DigitalInput limitSwitchB;
  private PIDController riderPID;

  /**
   * Initializes all Rider components
   */
  private Rider() {

    angleMotor = new WPI_TalonSRX(RobotMap.ANGLE_MOTOR);
    intakeMotor = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR);

    encoderAngle = new Encoder(RobotMap.ENCODER_PORT_RIDER_ONE, RobotMap.ENCODER_PORT_RIDER_TWO, false,
        EncodingType.k4X);
    encoderAngle.setDistancePerPulse(1);

    limitSwitchA = new DigitalInput(RobotMap.RIDER_LIMIT_SWITCH_ANGLE_PORT);
    limitSwitchB = new DigitalInput(RobotMap.RIDER_LIMIT_SWITCH_INTAKE_PORT);

    riderPID = new PIDController(1, 1, 1, encoderAngle, angleMotor);
    encoderAngle.setPIDSourceType(PIDSourceType.kDisplacement);
  }

  public void PIDRiderEnable() {
    riderPID.enable();
  }

  public void setSetPointRider(double setPoint) {
    riderPID.setSetpoint(setPoint);
  }

  /**
   * Disables the PID controller
   */
  public void PIDDisable() {
    riderPID.disable();
  }

  /**
   * Set the set point range
   * 
   * @param tolerance The given range
   */
  public void setPIDTolerance(double tolerance) {
    riderPID.setAbsoluteTolerance(tolerance);
  }

  /**
   * Set the PID destination (set point)
   * 
   * @param setPoint The given destination
   */
  public void setPointRider(double setPoint) {
    riderPID.setSetpoint(setPoint);

  }

  /**
   * 
   * Return if the robot reached the desire destination
   * 
   * @return Indication if rider is on target
   */
  public boolean isRiderOnTarget() {
    return riderPID.onTarget();
  }

  /**
   * Control the intake motor power
   * 
   * @param speed The given power
   */
  public void controlIntakeMotor(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Control the angle motor power
   * 
   * @param angleSpeed The given power
   */
  public void controlAngleMoter(double angleSpeed) {
    angleMotor.set(ControlMode.PercentOutput, angleSpeed);
  }

  public boolean isLimitSwitchAnglePressed() {
    return limitSwitchA.get() || limitSwitchB.get();
  }

  public boolean isAngleInRange(double angle, double tolerance) {
    return encoderAngle.get() < angle + tolerance && encoderAngle.get() > angle - tolerance;
  }

  public int getCurrentAngle() {
    return encoderAngle.get();
  }

  /**
   * Reset the encoder
   */
  public void encoderReset() {
    encoderAngle.reset();
  }

  /**
   * Makes sure that only one intance runs at a time
   * 
   * @return Return the instance
   */
  public static Rider getInstance() {
    if (instance == null) {
      instance = new Rider();
    }

    return instance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here
  }
}
