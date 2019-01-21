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

  public static Rider instance;

  private WPI_TalonSRX angleMotor;
  private WPI_TalonSRX intakeMotor;
  private Encoder encoder;
  private DigitalInput limitSwitchAngle;
  private DigitalInput limitSwitchIntake;
  private PIDController riderPID;



  public static final double KP_RIDER =0.0;
  public static final double kD_RIDER =0.0;
  public static final double kI_RIDER =0.0;
  public static final double DISTANCE_PER_PULSE = 0.1;


  /**
   * Initializes all Rider components
   */
  private Rider() {

    angleMotor = new WPI_TalonSRX(RobotMap.ANGLE_MOTOR);
    intakeMotor = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR);

    encoder = new Encoder(RobotMap.ENCODER_PORT_RIDER_ONE, RobotMap.ENCODER_PORT_RIDER_TWO, false, EncodingType.k4X);
    encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

    limitSwitchAngle = new DigitalInput(RobotMap.RIDER_LIMIT_SWITCH_ANGLE_PORT);
    limitSwitchIntake = new DigitalInput(RobotMap.RIDER_LIMIT_SWITCH_INTAKE_PORT);

    riderPID = new PIDController(KP_RIDER, kI_RIDER, kD_RIDER, encoder, angleMotor);
    encoder.setPIDSourceType(PIDSourceType.kDisplacement);
  }

  /**
   * Enables the PID controller
   */
  public void PIDRiderEnable() {
    riderPID.enable();
  }

  /**
   * Disables the PID controller
   */
  public void PIDDisable() {
    riderPID.disable();
  }
 
  /**
   * Set the set point range
   * @param TOLORANCE The given range
   */
  public void RiderPIDTolerance(double TOLORANCE) {
    riderPID.setAbsoluteTolerance(TOLORANCE);
  }

  /**
   * Set the PID destination (set point)
   * @param setPoint The given destination
   */
  public void setPointRider(double setPoint) {
    riderPID.setSetpoint(setPoint);

  }

   /**
    * 
    * Return if the robot reached the desire destination 
    * @return Indication if rider is on target
    */
  public boolean isRiderOnTarget() {
    return riderPID.onTarget();
  }

  /**
   * Control the intake motor power
   * @param speed The given power
   */
  public void controlIntakeMotor(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

    /**
     * Control the angle motor power
     * @param angleSpeed The given power
     */
  public void controlAngleMoter(double angleSpeed) {
    angleMotor.set(ControlMode.PercentOutput, angleSpeed);
  }
  
    /**
     * Return if limit switch is pressed or not
     * @return Indication if limit switch is pressed
     */
  public boolean isLimitSwitchAnglePressed() {
    return limitSwitchAngle.get() || limitSwitchIntake.get();
  }

    /**
     * Get the Rider current angle
     * @param angle The current angle
     * @return Return the current angle
     */
  public boolean getCurrentAngle(double angle) {
    return encoder.get() == angle;
  }
    
  /**
   * Reset the encoder
   */
  public void encoderReset() {
    encoder.reset();
  }
    
  /**
   *  Makes sure that only one intance runs at a time
   * @return Return the instance
   */
    public static Rider getInstance()  {
      if(instance == null){
          instance = new Rider();
      }

      return instance;
    }

  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here
  }
}
