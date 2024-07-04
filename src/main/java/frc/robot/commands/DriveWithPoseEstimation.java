// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonvisionPose;

public class DriveWithPoseEstimation extends Command {
  private DriveSubsystem drive;
  private DoubleSupplier xspeed;
  private DoubleSupplier yspeed;
  private DoubleSupplier rotspeed;
  private PhotonvisionPose visionPose;
  private EstimatedRobotPose leftPose;
  
  /** Creates a new drive. */
  public DriveWithPoseEstimation(
    DriveSubsystem drive,
    DoubleSupplier xspeed,
    DoubleSupplier yspeed,
    DoubleSupplier rotspeed,
    PhotonvisionPose visionPose) {
    
    this.visionPose=visionPose;
    this.drive=drive;
    this.xspeed=xspeed;
    this.yspeed=yspeed;
    this.rotspeed= rotspeed;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
      }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    EstimatedRobotPose leftPose=visionPose.getLeftEstimatedRobotPose();
    EstimatedRobotPose rightPose= visionPose.getRightEstimatedRobotPose();

    drive.addVisionMeasurement(leftPose.estimatedPose.toPose2d(), leftPose.timestampSeconds);
    drive.addVisionMeasurement(rightPose.estimatedPose.toPose2d(), rightPose.timestampSeconds);
    
    //TODO get pose estimations and feed to drive esimator
    drive.drive(
      MathUtil.applyDeadband(xspeed.getAsDouble(), OIConstants.kDriveDeadband),
      MathUtil.applyDeadband(yspeed.getAsDouble(), OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(rotspeed.getAsDouble(), OIConstants.kDriveDeadband),
      true, true);
    
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
