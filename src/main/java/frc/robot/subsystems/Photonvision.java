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
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Photonvision extends SubsystemBase {
  PhotonCamera Shootercamera = new PhotonCamera("photonvision");
  PhotonCamera rearCamera = new PhotonCamera("photonvision");
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  Transform3d cameraToRobot = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  Pose3d estiPose3d;
  /** Creates a new Photonvision. */
  public Photonvision() 
  {

  }
  public Pose3d getEsitmatedPose3d(){
    return estiPose3d;
  }

  @Override
  public void periodic() {
    var result=rearCamera.getLatestResult();
    if (result.hasTargets())
    {
     PhotonTrackedTarget target = result.getBestTarget();
     estiPose3d = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);
      
    }
    // This method will be called once per scheduler run
  }
}
