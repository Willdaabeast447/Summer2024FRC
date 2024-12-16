// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hoppper;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
  Shooter shooter;
  Hoppper hoppper;

  /** Creates a new SpinUP. */
  public Shoot(Shooter shooter,Hoppper hoppper) {
    this.shooter=shooter;
    this.hoppper=hoppper;
    addRequirements(shooter);
    addRequirements(hoppper);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.velocityControledShooterMotor(100);
    hoppper.setAgitator();
    hoppper.setElevator();
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
