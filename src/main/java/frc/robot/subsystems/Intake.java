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

  // motors
  private WPI_TalonSRX intakeTalon;
  private WPI_TalonSRX intakeAngleA;
  private WPI_TalonSRX intakeAngleB;

  // encoder
  private Encoder encoderIntake;

  // PID
  private PIDController anglePIDController;

  // Electric Pistons
  private Relay intakePistonRight;
  private Relay intakePistonLeft;

  private static Intake intakeSubsystem;

  // sensors
  private DigitalInput limitSwitchUp;
  private DigitalInput limitSwitchDown;

  private Intake() {

    limitSwitchUp = new DigitalInput(RobotMap.LIMIT_SWITCH_UP);
    limitSwitchDown = new DigitalInput(RobotMap.LIMIT_SWITCH_DOWN);

    intakePistonRight = new Relay(RobotMap.RELAY_PISTON_RIGHT);
    intakePistonLeft = new Relay(RobotMap.RELAY_PISTON_LEFT);

    intakeTalon = new WPI_TalonSRX(RobotMap.INTAKE_MOTORS_WHEELS);

    intakeAngleA = new WPI_TalonSRX(RobotMap.INTAKE_MOTORS_ANGLE_A);
    intakeAngleB = new WPI_TalonSRX(RobotMap.INTAKE_MOTORS_ANGLE_B);

    intakeAngleA.setInverted(true);
    intakeAngleB.set(ControlMode.Follower, intakeAngleA.getDeviceID());

    encoderIntake = new Encoder(RobotMap.ENCODER_B_INTAKE, RobotMap.ENCODER_A_INTAKE, false, EncodingType.k4X);
    encoderIntake.setDistancePerPulse(1);
    encoderIntake.setPIDSourceType(PIDSourceType.kDisplacement);

    anglePIDController = new PIDController(1, 1, 1, encoderIntake, intakeAngleA);
  }

  public boolean isLimitSwitchUp() {
    return limitSwitchUp.get();
  }

  public boolean isLimitSwitchDown() {
    return limitSwitchDown.get();
  }

  public void enablePID() {
    anglePIDController.enable();
  }

  public void disablePID() {
    anglePIDController.disable();
  }

  public void setSetpointPID(double setSetpoint) {
    anglePIDController.setSetpoint(setSetpoint);
  }

  public void setTolerancePID(double Tolerance) {
    anglePIDController.setAbsoluteTolerance(Tolerance);
  }

  public boolean isOnTargetPID() {
    return anglePIDController.onTarget();
  }

  public void intakeControl(double speed) {
    intakeTalon.set(ControlMode.PercentOutput, speed);
  }

  public void intakeMovmentControl(double speedUpAndDown) {
    intakeAngleA.set(ControlMode.PercentOutput, speedUpAndDown);
  }

  public void RelayControlFowerd() {
    intakePistonRight.set(Relay.Value.kForward);
    intakePistonLeft.set(Relay.Value.kForward);
  }

  public void RelayControlRevers() {
    intakePistonRight.set(Relay.Value.kReverse);
    intakePistonLeft.set(Relay.Value.kReverse);
  }

  public void resetEncoder() {
    encoderIntake.reset();
  }

  public static Intake getInstance() {
    if (intakeSubsystem == null) {
      intakeSubsystem = new Intake();
    }
    return intakeSubsystem;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
