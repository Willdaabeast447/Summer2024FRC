// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Photonvision extends SubsystemBase {
  PhotonCamera camera;
  AprilTagFieldLayout field;
  Transform3d cameraToRobot; //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  Pose3d estiPose3d;
   PhotonTrackedTarget target;
  private boolean validTarget;
  /** Creates a new Photonvision. */
  public Photonvision(String camerashooter, AprilTagFieldLayout field,Transform3d cameraToRobot) 
  {
    this.field=field;
    this.camera=new PhotonCamera(camerashooter);
    this.cameraToRobot=cameraToRobot;
  }
  public Pose3d getEsitmatedPose3d(){
    return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), field.getTagPose(target.getFiducialId()).get(), cameraToRobot);
  }
  public void targetFound(){

  }
  public int getTargetID(){
    return this.target.getFiducialId();
  }

  public double getTargetYaw()
  {
    SmartDashboard.putNumber("tag yaw",target.getYaw());
    return this.target.getYaw();
    
  }
  public boolean hasTargets()
  {
    return this.validTarget;
  }

  @Override
  public void periodic() {
    var result=camera.getLatestResult();
    this.validTarget=result.hasTargets();
    if (result.hasTargets())
    {
     
     this.target = result.getBestTarget();
    }
    // This method will be called once per scheduler run
  }
}
