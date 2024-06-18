// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class drive extends Command {
  private DriveSubsystem drive;
  private DoubleSupplier xspeed;
  private DoubleSupplier yspeed;
  private DoubleSupplier rotspeed;
  private BooleanSupplier xModules;

  /** Creates a new drive. */
  public drive(DriveSubsystem drive,DoubleSupplier xspeed,DoubleSupplier yspeed, DoubleSupplier rotspeed,BooleanSupplier xModules) {
    
    this.drive=drive;
    this.xspeed=xspeed;
    this.yspeed=yspeed;
    this.rotspeed= rotspeed;
    this.xModules=xModules;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
      }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (xModules.getAsBoolean())
    {
      drive.setX();
    }
    else{
    drive.drive(
      -MathUtil.applyDeadband(xspeed.getAsDouble(), OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(yspeed.getAsDouble(), OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(rotspeed.getAsDouble(), OIConstants.kDriveDeadband),
      true, true);
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
