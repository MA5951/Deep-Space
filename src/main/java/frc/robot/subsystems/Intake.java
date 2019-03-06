/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * The intake subsystem
 */
public class Intake extends Subsystem {

  // motors
  private WPI_VictorSPX intakeBallMotor;
  private WPI_TalonSRX intakeAngleMotorA;
  private WPI_TalonSRX intakeAngleMotorB;

  // encoder
  private Encoder encoderIntake;

  // PID
  private PIDController anglePID;

  // Pneomatic Pistons
  private Solenoid intakePiston;

  private static Intake i_Instance;

  public static final double KP_ENCODER = 0.005;
  public static final double KI_ENCODER = 0.0005;
  public static final double KD_ENCODER = 0.007;
  private static final double DISTANCE_PER_PULSE = 1;
  public static final double TOLERANCE = 20;

  /**
   * Initializes all Chassis components
   */
  private Intake() {

    intakePiston = new Solenoid(RobotMap.PCM, RobotMap.INTAKE_PISTON_FORWARD);

    intakeBallMotor = new WPI_VictorSPX(RobotMap.INTAKE_MOTORS_WHEELS);

    intakeAngleMotorA = new WPI_TalonSRX(RobotMap.INTAKE_MOTORS_ANGLE_LEFT_A);
    intakeAngleMotorB = new WPI_TalonSRX(RobotMap.INTAKE_MOTORS_ANGLE_RIGHT_B);

    intakeAngleMotorB.setInverted(true);
    intakeAngleMotorB.set(ControlMode.Follower, intakeAngleMotorA.getDeviceID());

    encoderIntake = new Encoder(RobotMap.INTAKE_ENCODER_A, RobotMap.INTAKE_ENCODER_B, false, EncodingType.k4X);
    encoderIntake.setDistancePerPulse(DISTANCE_PER_PULSE);
    encoderIntake.setPIDSourceType(PIDSourceType.kDisplacement);

    anglePID = new PIDController(KP_ENCODER, KI_ENCODER, KD_ENCODER, encoderIntake, intakeAngleMotorA);
    anglePID.setAbsoluteTolerance(TOLERANCE);
    anglePID.setOutputRange(-0.85, 0.85);

  }

  public void intakeSmartdashboardValue() {
    SmartDashboard.putNumber("Intake Encoder", encoderIntake.getDistance());
    SmartDashboard.putBoolean("Intake Piston", intakePiston.get());
  }

  /**
   * Check whether the limitswitch is pressed
   * 
   * @return Indication if the limitswitch is pressed.
   */
  public boolean isIntakeLimitswitchClosed() {
    return intakeAngleMotorA.getSensorCollection().isFwdLimitSwitchClosed();
  }

  /**
   * Enables the PIDController
   */
  public void enablePID(boolean enable) {
    if (enable) {
      anglePID.enable();
    } else {
      anglePID.disable();
    }
  }


  /**
   * Set the PIDController destination (setpoint)
   *
   * @param setpoint The given destination (setpoint)
   */
  public void setSetpoint(double setpoint) {
    anglePID.setSetpoint(setpoint);
  }

  /**
   * Check whether {anglePIDController} is on target
   * 
   * @return Indication if {anglePIDController} is on target
   */
  public boolean isPIDOnTarget() {
    return anglePID.onTarget();
  }

  /**
   * Give power to the intake motors
   * 
   * @param speed The given power
   */
  public void intakeBallControl(double speed) {
    intakeBallMotor.set(speed);
  }

  /**
   * Give power to the angleA motor
   * 
   * @param speedUpAndDown The given power
   */
  public void intakeAngleControl(double speedUpAndDown) {
    intakeAngleMotorA.set(ControlMode.PercentOutput, speedUpAndDown);
  }

  public boolean isEncoderInDistanceRangeIntake(double maxDistance, double minDistance) {
    return encoderIntake.getDistance() <= maxDistance && encoderIntake.getDistance() >= minDistance;
  }

  public double getEncoder() {
    return encoderIntake.getDistance();
  }

  /**
   * Give power to the pistons (up).
   */
  public void pistonControlForward() {
    intakePiston.set(true);

  }

  /**
   * Turn off the pistons.
   */
  public void pistonControlOff() {
    intakePiston.set(false);

  }

  /**
   * Reset the encoder.
   */
  public void resetEncoder() {
    encoderIntake.reset();
  }

  public double getPIDMotorsSpeed() {
    return anglePID.get();
  }

  /**
   * Makes sure that only one instance runs at a time
   * 
   * @return Return the instance
   */
  public static Intake getInstance() {
    if (i_Instance == null) {
      i_Instance = new Intake();
    }
    return i_Instance;
  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand();
  }
}
