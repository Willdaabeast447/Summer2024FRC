// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.Constants.ShooterConstants;;
public class Shooter extends SubsystemBase {
  private final TalonSRX elevatorTalonFX= new TalonSRX(ShooterConstants.kElevatorId);
  private final TalonSRX funnelTalonFX= new TalonSRX(ShooterConstants.kFunnelId);
  private final TalonSRX floorFx= new TalonSRX(ShooterConstants.kFloorId);
  private final TalonFX shooterLeftFx= new TalonFX(ShooterConstants.kShooterLeftId);
  private final TalonFX shooterRightFx = new TalonFX(ShooterConstants.kShooterRightId);
  private final TalonFX shooterRotationFx= new TalonFX(ShooterConstants.kShooterRotationId);
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  private SlewRateLimiter turretRotLimiter = new SlewRateLimiter(ShooterConstants.kRotationalSlewRate);
  private double tartgetTurretPosition=0;
  private boolean enableTurretPID=false;
  private double TurretmotorValue=0;
  /** Creates a new Shooter. */
  public Shooter() {
 shooterRotationFx.getPosition() ; 

  TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 1; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.0001; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;
    shooterRotationFx.getConfigurator().apply(configs);
    shooterLeftFx.setInverted(true);
    shooterRightFx.setInverted(false);
    shooterRotationFx.setPosition(0.0);

    
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
    if( this.enableTurretPID){
      this.enableTurretPID=false;
    }
    this.TurretmotorValue=motorValue;

  }
  /*
   * using built in pid on falcon 500 drive to postion
   */
  public void driveTurretToPos(double position){
    if( !this.enableTurretPID){
      this.enableTurretPID=true;
    }
    double postionAdjustedInRange= shooterSetpointInRotationRange(position);
    double rotations= postionAdjustedInRange*ShooterConstants.turretRotPerDeg;
    this.tartgetTurretPosition=rotations;
  }

  /*
   *manual rotation soft limit function
   takes current motor postion and a control value and if motion is withing limits
   allows for normal use
   if motion is outside of limits then only allow values that would drive withing limits. 
   */
  public double turretRotationlimits(double position,double motorValue)
  {
    double output=0;
    if ( ShooterConstants.turretPositionFwdLimit>position&& ShooterConstants.turretPositionRevLimit<position)
    {
    output=motorValue;
    }
    else if (ShooterConstants.turretPositionFwdLimit<=position)
    {
      if( motorValue<0)
      {
         output=motorValue; 
      }
      else{
        output =0;
      }

    }
    else if (ShooterConstants.turretPositionRevLimit>=position)
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
 * shooterSetpointIn RotationRange Function
 * Takes a degree value in as a double and checks if it is 
 * in rotation rang of + or - the trurret rotation degree limit 
 * defined in the Shooter Constants class in the constants file.
 * If Setpoint is oustide that range converts it to a setpoint 
 * with oposite rotation 
 */
  public double shooterSetpointInRotationRange(double setpoint)
  {
    double output;
    if (Math.abs(setpoint)>ShooterConstants.turretRotationLimitDeg)
    {
      double theta= Math.abs(setpoint)-ShooterConstants.turretRotationLimitDeg;
      if (setpoint>ShooterConstants.turretRotationLimitDeg){
        output=-ShooterConstants.turretRotationLimitDeg+theta;        
      }
      else{
        output=ShooterConstants.turretRotationLimitDeg-theta;
      }
    }
    else{
      output=setpoint;
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
    if(enableTurretPID){
      shooterRotationFx.setControl(
        m_positionVoltage.withPosition(turretRotLimiter.calculate(tartgetTurretPosition)));
    }
    else
    {
      shooterRotationFx.set(turretRotationlimits(getTurretPosition(), TurretmotorValue));
    }
  }
}
