// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photonvision extends SubsystemBase {
  PhotonCamera Shootercamera = new PhotonCamera("photonvision");
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  /** Creates a new Photonvision. */
  public Photonvision() {}

  @Override
  public void periodic() {
    var result=Shootercamera.getLatestResult()
    if (result.hasTargets())
    {
     PhotonTrackedTarget target = result.getBestTarget();
      
    }
    // This method will be called once per scheduler run
  }
}
