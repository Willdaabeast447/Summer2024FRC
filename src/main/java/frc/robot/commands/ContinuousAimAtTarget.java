// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonvisionTurret;
import frc.robot.subsystems.Shooter;

public class ContinuousAimAtTarget extends Command {
  private PhotonvisionTurret sight;
  private Shooter shooter;
  private double yaw=0;
  private DriveSubsystem drive;
  private Pose2d pose=new Pose2d();
  private int targetTag;
  private Pose3d tagPose;


  /** Creates a ContinuousAimAtTarget */
  public ContinuousAimAtTarget(PhotonvisionTurret sight,Shooter kshooter,DriveSubsystem drive,int targetTag) {
    this.sight=sight;
    this.shooter=kshooter;
    this.drive=drive;
    this.tagPose=VisionConstants.field.getTagPose(targetTag).get();
    this.targetTag=targetTag;
    addRequirements(sight);
    addRequirements(shooter);
    
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

 

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pose=drive.getPose();
    double robotRotation=pose.getRotation().getDegrees()>0?pose.getRotation().getDegrees():360+pose.getRotation().getDegrees();
    SmartDashboard.putNumber("rotation pose ", robotRotation);
    PhotonTrackedTarget target = sight.lookForTag(targetTag);
    if(target!=null){
       this.yaw = -target.getYaw();
       shooter.autoAimTurret(yaw);
      }
      else{
        
        double deltaY= -pose.getY()+tagPose.getY();
        SmartDashboard.putNumber("deltay", deltaY);
        double deltaX =pose.getX()-tagPose.getX();
        SmartDashboard.putNumber("deltax", deltaX);
        double angleTheta=Math.toDegrees(Math.atan(deltaX/deltaY));
        SmartDashboard.putNumber("Angle theta", angleTheta);
        double angleBeta=90-Math.abs(angleTheta);
        SmartDashboard.putNumber("angle beta", angleBeta);
        double angleAlpha;
        if (deltaY<0){
          if (deltaX>0){
          angleAlpha=180+angleBeta;
          }
          else{
          angleAlpha=360-angleBeta;
          }
        }
        else{
          if (deltaX>0){
          angleAlpha=180-angleBeta;
          }
          else{
          angleAlpha=angleBeta;
          }

        }
         SmartDashboard.putNumber("angle alpha", angleAlpha);   
        double turretAngle=angleAlpha-robotRotation;
        SmartDashboard.putNumber("turret aiming angle", turretAngle);
        shooter.driveTurretToPos(turretAngle);
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
