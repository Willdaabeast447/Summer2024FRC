// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Shooter;

public class TESTshooter extends Command {

  private Shooter shootersub;
  private DoubleSupplier yspeed;
  private Double rotspeed;
  private BooleanSupplier leftBumper;
  private BooleanSupplier rightBumper;
  

  /** Creates a new TESTshooter. */
  public TESTshooter(Shooter shooterSubsystem,DoubleSupplier yspeed, BooleanSupplier leftbumper,BooleanSupplier rightbumper) {
  
    this.yspeed=yspeed;   
    this.shootersub=shooterSubsystem;
    this.leftBumper=leftbumper;
    this.rightBumper= rightbumper;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(leftBumper.getAsBoolean()||rightBumper.getAsBoolean()&& !(leftBumper.getAsBoolean()&&rightBumper.getAsBoolean()))
    {
      if (leftBumper.getAsBoolean())
      {
        rotspeed=-0.2;
      }
      else
      {
        rotspeed=0.2;
      }
    }
    
    else
    {
      rotspeed=0.0;
    }
    shootersub.testMotors(
      -MathUtil.applyDeadband(yspeed.getAsDouble(), OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(this.rotspeed, OIConstants.kDriveDeadband));
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
