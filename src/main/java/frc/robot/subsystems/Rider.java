/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Rider.AngleInwards;
import frc.robot.commands.Rider.AngleOutward;
import frc.robot.commands.Rider.IntakeIn;
import frc.robot.commands.Rider.IntakeOut;
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
  private DigitalInput limitSwitchAngle;
  private DigitalInput limitSwitchIntake;
  


  public Rider(){
    
    angleMotor = new WPI_TalonSRX(RobotMap.ANGLE_MOTOR);
    intakeMotor = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR);

    encoder = new Encoder(RobotMap.ENCODER_PORT_RIDER_ONE, RobotMap.ENCODER_PORT_RIDER_TWO);
    encoder.reset();

    limitSwitchAngle = new DigitalInput(RobotMap.RIDER_LIMIT_SWITCH_ANGLE_PORT);
    limitSwitchIntake = new DigitalInput(RobotMap.RIDER_LIMIT_SWITCH_INTAKE_PORT);


  }


    public void intakeIn()  {
      intakeMotor.set(-0.5);
    }

    public void intakeOut() {
      intakeMotor.set(0.5);
    }


    public void stopIntake()  {
      intakeMotor.set(0);
    }
  
    public boolean isLimitSwitchAnglePressed(){
      return limitSwitchAngle.get();
    }

    public void angleInward()  {
      angleMotor.set(-0.5);
    }

    public void angleOutward()  {
      angleMotor.set(0.5);
    }


    public void stopAngleMotor(){
      angleMotor.set(0);
    }


    public int getCurrentAngle(){
      return encoder.get();
    }
    
    public boolean isLimitSwitchIntakePressed(){
      return limitSwitchIntake.get();
    }

    public static Rider getInstance()  {
      if(instance == null){
          instance = new Rider();
      }

      return instance;
    }
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new IntakeIn());
    setDefaultCommand(new IntakeOut());

    //setDefaultCommand(new AngleInwards());
    //setDefaultCommand(new AngleOutward());

  }
}
