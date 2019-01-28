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
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.intake.StopIntakeMovement;

/**
 * The intake subsystem
 */
public class Intake extends Subsystem {

  // motors
  private WPI_TalonSRX intakeBallMotor; 
  private WPI_TalonSRX intakeAngleMotorA; 
  private WPI_TalonSRX intakeAngleMotorB; 

  // encoder
  private Encoder encoderIntake;

  // PID
  private PIDController anglePID; 

  // Pneomatic Pistons
  private DoubleSolenoid intakePistonRight;
  private DoubleSolenoid intakePistonLeft;

  private static Intake i_Instance;

  // sensors
  private DigitalInput limitSwitchUp;
  private DigitalInput limitSwitchDown;

  public static final double KP_ENCODER = 0.0;
  public static final double KI_ENCODER = 0.0;
  public static final double KD_ENCODER = 0.0;
  private static final double DISTANCE_PER_PULSE = 1;

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

    intakeBallMotor = new WPI_TalonSRX(RobotMap.INTAKE_MOTORS_WHEELS);

    intakeAngleMotorA = new WPI_TalonSRX(RobotMap.INTAKE_MOTORS_ANGLE_A);
    intakeAngleMotorB = new WPI_TalonSRX(RobotMap.INTAKE_MOTORS_ANGLE_B);

    // TODO Check if intakeAngleB needs to be inverted
    intakeAngleMotorA.setInverted(true);
    intakeAngleMotorB.set(ControlMode.Follower, intakeAngleMotorA.getDeviceID());

    encoderIntake = new Encoder(RobotMap.INTAKE_ENCODER_A, RobotMap.INTAKE_ENCODER_B, false, EncodingType.k4X);
    encoderIntake.setDistancePerPulse(DISTANCE_PER_PULSE);
    encoderIntake.setPIDSourceType(PIDSourceType.kDisplacement);

    anglePID = new PIDController(KP_ENCODER, KI_ENCODER, KD_ENCODER, encoderIntake, intakeAngleMotorA);
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
   * Set the PIDController range
   * 
   * @param Tolerance The given range
   */

  /**
   * Check whether {anglePIDController} is on target
   * 
   * @return Indication if {anglePIDController} is on target
   */
  public boolean isOnTarget() { 
    return anglePID.onTarget();
  }

  /**
   * Give power to the intake motors
   * 
   * @param speed The given power
   */
  public void intakeControl(double speed) {
    intakeBallMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Give power to the angleA motor
   * 
   * @param speedUpAndDown The given power
   */
  public void intakeAngleControl(double speedUpAndDown) {
    intakeAngleMotorA.set(ControlMode.PercentOutput, speedUpAndDown);
  }

  /**
   * Give power to the pistons (up).
   */
  @Deprecated
  public void PistonControlForward() {
    intakePistonRight.set(Value.kForward);
    intakePistonLeft.set(Value.kForward);
  }

  /**
   * Give power to the pistons (down)
   */
  public void PistonControlReverse() {
    intakePistonRight.set(Value.kReverse);
    intakePistonLeft.set(Value.kReverse);
  }

  /**
   * Turn off the pistons.
   */
  public void PistonControlOff() { // TODO Add javadoc
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
  public void initDefaultCommand() { // Add default command that does nothing except stop intake movement
    setDefaultCommand(new StopIntakeMovement());
  }
}
