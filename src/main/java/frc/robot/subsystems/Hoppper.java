// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Hoppper extends SubsystemBase {
  private final TalonSRX elevatorTalonFX= new TalonSRX(ShooterConstants.kElevatorId);  
  private final TalonSRX agitatorFx= new TalonSRX(ShooterConstants.kFloorId);

  /* 
   * shuffleboard setup
   */
  private ShuffleboardTab tab = Shuffleboard.getTab("Hopper");
   private GenericEntry RHB_Speed =
      tab.add("RHB Speed", 0)
         .getEntry();
    private GenericEntry elevator_Speed =
      tab.add("Elevator Speed", 0)
         .getEntry();
  /** Creates a new Hoppper. */
  public Hoppper() {
    agitatorFx.setInverted(false);

  }
  
  // set the speed of the Rotary Ball Hopper
  public void setAgitator(){
    agitatorFx.set(TalonSRXControlMode.PercentOutput,RHB_Speed.getDouble(0));
  }
  // stop the Rotary Ball Hopper
  public void stopAgitator(){
    agitatorFx.set(TalonSRXControlMode.PercentOutput,0);
  }
  
  // set the speed of the elevator
  public void setElevator(){
    elevatorTalonFX.set(TalonSRXControlMode.PercentOutput, elevator_Speed.getDouble(0));
  }
  // stop the elevator
  public void stopElevator(){
    elevatorTalonFX.set(TalonSRXControlMode.PercentOutput,0);
  }
  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
