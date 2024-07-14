// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hoppper;
import frc.robot.subsystems.Shooter;

public class TestToggle extends Command {
  private  Shooter shooter;
  private double position;
  private Hoppper hopper;
  private DoubleSupplier speed;
  /** Creates a new TestToggle. */
  public TestToggle(Shooter shooterSubsystem,Hoppper hoppper ,double position,DoubleSupplier speed) {
    this.shooter=shooterSubsystem;
    this.position=position;
    this.hopper=hoppper;
    this.speed=speed;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.driveTurretToPos(position);
    if (Math.abs(speed.getAsDouble())>0)
    {
    hopper.setAgitator(0.5);
    hopper.setElevator(1);
    shooter.setShooterMotor(speed.getAsDouble());
    }
    else
    {
    hopper.setAgitator(0.0);
    hopper.setElevator(0);
    shooter.setShooterMotor(speed.getAsDouble());
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
