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
  private ShuffleboardTab tab = Shuffleboard.getTab("Hopper");
   private GenericEntry RHB_Speed =
      tab.add("RHB Speed", 0)
         .getEntry();
  /** Creates a new Hoppper. */
  public Hoppper() {
    agitatorFx.setInverted(false);

  }

  public void setAgitator(){
    agitatorFx.set(TalonSRXControlMode.PercentOutput,RHB_Speed.getDouble(0));
  }
  public void stopAgitator(){
    agitatorFx.set(TalonSRXControlMode.PercentOutput,0);
  }

   public void setElevator(double speed){
    elevatorTalonFX.set(TalonSRXControlMode.PercentOutput, speed);
  }
  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
