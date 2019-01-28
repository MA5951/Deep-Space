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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * The intake subsystem
 */
public class Intake extends Subsystem {

  // motors
  private WPI_TalonSRX intakeBall;
  private WPI_TalonSRX intakeAngleA;
  private WPI_TalonSRX intakeAngleB;

  // encoder
  private Encoder encoderIntake;

  // PID
  private PIDController anglePIDController;

  // Pneomatic Pistons
  private DoubleSolenoid intakePistonRight;
  private DoubleSolenoid intakePistonLeft;

  private static Intake i_Instance;

  // sensors
  private DigitalInput limitSwitchUp;
  private DigitalInput limitSwitchDown;

  /**
   * Initializes all Chassis components
   */
  private Intake() {

    limitSwitchUp = new DigitalInput(RobotMap.INTAKE_LIMIT_SWITCH_UP);
    limitSwitchDown = new DigitalInput(RobotMap.INTAKE_LIMIT_SWITCH_DOWN);

    intakePistonRight = new DoubleSolenoid(RobotMap.PCM, RobotMap.INTAKE_PISTON_RIGHT_FORWARD,
        RobotMap.INTAKE_PISTON_RIGHT_BACKWARD);
    intakePistonLeft = new DoubleSolenoid(RobotMap.PCM, RobotMap.INTAKE_PISTON_LEFT_FORWARD,
        RobotMap.INTAKE_PISTON_LEFT_BACKWARD);

    intakeBall = new WPI_TalonSRX(RobotMap.INTAKE_MOTORS_WHEELS);

    intakeAngleA = new WPI_TalonSRX(RobotMap.INTAKE_MOTORS_ANGLE_A);
    intakeAngleB = new WPI_TalonSRX(RobotMap.INTAKE_MOTORS_ANGLE_B);

    //Check if intakeAngleB need to be inverted
    intakeAngleA.setInverted(true);
    intakeAngleB.set(ControlMode.Follower, intakeAngleA.getDeviceID());

    encoderIntake = new Encoder(RobotMap.INTAKE_ENCODER_B, RobotMap.INTAKE_ENCODER_A, false, EncodingType.k4X);
    encoderIntake.setDistancePerPulse(1);
    encoderIntake.setPIDSourceType(PIDSourceType.kDisplacement);

    anglePIDController = new PIDController(1, 1, 1, encoderIntake, intakeAngleA);
  }

  /**
   * Check whether limit switch is pressed
   * 
   * @return Indication if {limitSwitchUp} is pressed
   */
  public boolean isLimitSwitchUpPressed() {
    return limitSwitchUp.get();
  }

  /**
   * Check whether limit switch is pressed
   * 
   * @return Indication if {limitSwitchDown} is pressed
   */
  public boolean isLimitSwitchDownPressed() {
    return limitSwitchDown.get();
  }

  /**
   * Enables the PIDController
   */
  public void enablePID() {
    anglePIDController.enable();
  }

  /**
   * Disables the PIDController
   */
  public void disablePID() {
    anglePIDController.disable();
  }

  /**
   * Set the PIDController destination (setpoint)
   *
   * @param setpoint The given destination (setpoint)
   */
  public void setSetpointPID(double setpoint) {
    anglePIDController.setSetpoint(setpoint);
  }

  /**
   * Set the PIDController range
   * 
   * @param Tolerance The given range
   */
  public void setTolerancePID(double Tolerance) {
    anglePIDController.setAbsoluteTolerance(Tolerance);
  }

  /**
   * Check whether {anglePIDController} is on target
   * 
   * @return Indication if {anglePIDController} is on target
   */
  public boolean isOnTargetPID() {
    return anglePIDController.onTarget();
  }

  /**
   * Give power to the intake motors
   * 
   * @param speed The given power
   */
  public void intakeControl(double speed) {
    intakeBall.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Give power to the angleA motor
   * 
   * @param speedUpAndDown The given power
   */
  public void intakeMovmentControl(double speedUpAndDown) {
    intakeAngleA.set(ControlMode.PercentOutput, speedUpAndDown);
  }

  /**
   * Give power to the pistons (up). TODO Fix, intake is no longer relay
   */
  @Deprecated
  public void PistonControlForward() {
    intakePistonRight.set(Value.kForward);
    intakePistonLeft.set(Value.kForward);
  }

  /**
   * Give power to the pistons (down) TODO Fix, intake is no longer relay
   */
  public void PistonControlReverse() {
    intakePistonRight.set(Value.kReverse);
    intakePistonLeft.set(Value.kReverse);
  }

  public void PistonControlOff() {
    intakePistonRight.set(Value.kOff);
    intakePistonLeft.set(Value.kOff);
  }

  /**
   * Reset the encoder.
   */
  public void resetEncoder() {
    encoderIntake.reset();
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
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
