// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final TalonSRX elevatorTalonFX= new TalonSRX(Constants.kElevatorId);
  private final TalonSRX funnelTalonFX= new TalonSRX(Constants.kFunnelId);
  private final TalonSRX floorFx= new TalonSRX(Constants.kFloorId);
  private final TalonFX shooterLeftFx= new TalonFX(Constants.kShooterLeftId);
  private final TalonFX shooterRightFx = new TalonFX(Constants.kShooterRightId);
  private final TalonFX shooterRotationFx= new TalonFX(Constants.kShooterRotationId);
  /** Creates a new Shooter. */
  public Shooter() {
 shooterRotationFx.getPosition() ; 

  TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;
    shooterRotationFx.getConfigurator().apply(configs);
    shooterLeftFx.setInverted(true);
    shooterRightFx.setInverted(false);

    
  }

  /*
   * Getter fuctions 
   * should only return single value
   * 
   */
  public double getTurretPosition()
  {
    return shooterRotationFx.getPosition().getValueAsDouble();
  }

  /*
   * Setter Funtions 
   * should only set single value
   * 
   */

  public void setShooterRPM(double rpm )
  {

  }

  public void setShooter(double speed)
  {

  }

  /*
   * Turrent funtions 
   */
  public void driveTurret(double motorValue)
  {
    shooterRotationFx.set(turretRotationlimits(getTurretPosition(), motorValue));

  }

  public double turretRotationlimits(double position,double motorValue)
  {
    double output=0;
    if ( Constants.turretPositionFwdLimit>position&& Constants.turretPositionRevLimit<position)
    {
    output=motorValue;
    }
    else if ( Constants.turretPositionFwdLimit<=position)
    {
      if( motorValue<0)
      {
         output=motorValue; 
      }
      else{
        output =0;
      }

    }
    else if ( Constants.turretPositionRevLimit>=position)
    {
      if( motorValue>0)
      {
         output=motorValue; 
      }
      else{
        output =0;
      }

    }
    return output;

  }

  /*
   * shooter functions
   */
  void setShooterMotor(double motorValue)
  {
    shooterLeftFx.set(motorValue);
    shooterRightFx.set(motorValue);
  }

  /*
   * hopper funtions
   */
  void feedBall(boolean feed)
  {
    if (feed){
     floorFx.set(ControlMode.PercentOutput, -0.95); 
     funnelTalonFX.set(ControlMode.PercentOutput, 0.95);
    }
    else{
    floorFx.set(ControlMode.PercentOutput, 0); 
    funnelTalonFX.set(ControlMode.PercentOutput, 0);
    }
  }
  
  /*
   * other funtions
   */
  public void aimAt(double goal, double setpoint)
  {

  }

  public void testMotors(double speed,double rot){
  elevatorTalonFX.set(ControlMode.PercentOutput, -speed);  
  if(Math.abs(speed)>0){
    feedBall(true);
  }
  else{
    feedBall(false);
  }
  setShooterMotor(-speed/3);
  driveTurret(rot);

  }

  @Override
  public void periodic() {

  }
}
