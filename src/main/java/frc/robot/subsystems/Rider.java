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
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;


import frc.robot.RobotMap;
/**
 * Add your docs here.
 */

public class Rider extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public static Rider instance;

  private WPI_TalonSRX angleMotor;
  private WPI_TalonSRX intakeMotor;
  private Encoder encoder;
  private DigitalInput limitSwitchA;
  private DigitalInput limitSwitchB;
  private PIDController riderPID;





  private Rider(){
    
    angleMotor = new WPI_TalonSRX(RobotMap.ANGLE_MOTOR);
    intakeMotor = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR);

    encoder = new Encoder(RobotMap.ENCODER_PORT_RIDER_ONE, RobotMap.ENCODER_PORT_RIDER_TWO,false,EncodingType.k4X);
    encoder.setDistancePerPulse(1);

    limitSwitchA = new DigitalInput(RobotMap.RIDER_LIMIT_SWITCH_ANGLE_PORT);
    limitSwitchB = new DigitalInput(RobotMap.RIDER_LIMIT_SWITCH_INTAKE_PORT);

    riderPID = new PIDController(1, 1, 1, encoder, angleMotor);
    encoder.setPIDSourceType(PIDSourceType.kDisplacement);
  }

public void PIDRiderEnable(){
  riderPID.enable();
}
public void PIDDisable(){
  riderPID.disable();
 }
public void RiderPIDTolerance(double tolorance){
  riderPID.setAbsoluteTolerance(tolorance);
}
public void setSetPointRider(double setPoint){
  riderPID.setSetpoint(setPoint);
}
public boolean isRiderOnTarget(){
  return riderPID.onTarget();
}

    public void controlIntakeMoter(double speed)  {
      intakeMotor.set(ControlMode.PercentOutput,speed);
    }

    public void controlAngleMoter(double angleSpped)  {
      angleMotor.set(ControlMode.PercentOutput,angleSpped);
    }
  
    public boolean isLimitSwitchAnglePressed(){
      return limitSwitchA.get()||limitSwitchB.get();
    }

    public boolean ifInTheCurrentAngle(double angle){
      return encoder.get()==angle;
    }
    public int getCurrentAngle(){
      return encoder.get();
    }
    
    public void encoderReset(){
      encoder.reset();
    }
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
