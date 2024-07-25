// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hoppper;
import frc.robot.subsystems.Shooter;

public class ManualShooter extends Command {
 
  private double motorvalue;
  private Shooter shooter;
  private DoubleSupplier rightTrigger;
  private BooleanSupplier rightBumper;
  private BooleanSupplier leftBumper;
  private Hoppper hopper;

  /** Creates a new ManualShooter. */
  public ManualShooter(Hoppper hopper,Shooter shooter,BooleanSupplier rightbumper,BooleanSupplier leftbumper, DoubleSupplier rightTrigger) {
    this.shooter=shooter;
    this.leftBumper=leftbumper;
    this.rightBumper=rightbumper;
    this.rightTrigger=rightTrigger;
    this.hopper=hopper;

    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed=rightTrigger.getAsDouble();
    if (rightBumper.getAsBoolean()||leftBumper.getAsBoolean())
    {
      if(rightBumper.getAsBoolean()){
        motorvalue=-0.1;
      }
      else
      {
        motorvalue=0.1;
      }
    }
    else
    {
      motorvalue=0;
    }
    shooter.driveTurret(motorvalue);
    shooter.setShooterMotor(rightTrigger.getAsDouble());
    if (Math.abs(speed)>0)
    {
    hopper.setAgitator();
    hopper.setElevator();
    }
    else
    {
    hopper.stopAgitator();
    hopper.stopElevator();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
