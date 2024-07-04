// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PhotonvisionPose extends SubsystemBase {
  PhotonCamera camera;
  AprilTagFieldLayout field;
  Transform3d cameraToRobot; //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  Optional<EstimatedRobotPose> estiPose3d;
   PhotonTrackedTarget target;
  private boolean validTarget;
  private PhotonPoseEstimator photonPoseEstimator;
  
  /** Creates a new Photonvision. */
  public PhotonvisionPose(String camerashooter, AprilTagFieldLayout field,Transform3d cameraToRobot) 
  {
    this.field=field;
    this.camera=new PhotonCamera(camerashooter);
    this.cameraToRobot=cameraToRobot;
    photonPoseEstimator = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.camera, this.cameraToRobot);
    
  
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
  public Pose2d getPose2d(){
    if(!(this.estiPose3d==null))
    {
    return new Pose2d(this.estiPose3d.get().estimatedPose.getX(),
                      this.estiPose3d.get().estimatedPose.getY(),
                      this.estiPose3d.get().estimatedPose.getRotation().toRotation2d());
    }
    else
    {
      return new Pose2d();
    }                  
  }
  public boolean hasTargets()
  {
    return this.validTarget;
  }
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    
    return photonPoseEstimator.update();
}

    // TODO make method to look for specific tag id

  @Override
  public void periodic() {
    // check to see if camera is being used to calculate a pose
      SmartDashboard.putString("hi", getName());
      // if not find the best target and return it
      var result=camera.getLatestResult();
      this.validTarget=result.hasTargets();
      if (result.hasTargets())
      {
     //TODO change this to find specific target
      this.target = result.getBestTarget();
      var est=getEstimatedGlobalPose();
      if (!(est==null)){
      double x =estiPose3d.get().estimatedPose.getX();
      double y =estiPose3d.get().estimatedPose.getY();
      double Rot =estiPose3d.get().estimatedPose.getRotation().getAngle();
      SmartDashboard.putNumber("pose X"+getName(),x);
      SmartDashboard.putNumber("pose Y"+getName(),y);
      SmartDashboard.putNumber("pose Rot"+getName(),Rot);
      }
      }
    }
    // This method will be called once per scheduler run
  }

