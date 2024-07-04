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
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;


public class PhotonvisionPose extends SubsystemBase {
  private PhotonCamera leftPhotonCamera=new PhotonCamera(VisionConstants.CameraOne);
  private PhotonCamera rightPhotonCamera=new PhotonCamera(VisionConstants.CameraTwo);
  private AprilTagFieldLayout field=VisionConstants.field;
  private Transform3d leftcameraToRobot=VisionConstants.LeftCamtobot;
  private Transform3d rightcameraToRobot=VisionConstants.RightCamtobot; //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  private PhotonPoseEstimator leftPhotonPoseEstimator = new PhotonPoseEstimator(this.field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,this.leftPhotonCamera,this.leftcameraToRobot);
  private PhotonPoseEstimator rightPhotonPoseEstimator = new PhotonPoseEstimator(this.field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,this.rightPhotonCamera,this.rightcameraToRobot);
  private Optional<EstimatedRobotPose> estiPose3d;

  private PhotonTrackedTarget target;
  private boolean validTarget;
  
  
  /** Creates a new Photonvision. */
  public PhotonvisionPose() 
  {
   
    
  
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
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator poseEstimator) {
    
    return poseEstimator.update();
}

    // TODO make method to look for specific tag id

  @Override
  public void periodic() {
    
      
      // if not find the best target and return it
      var leftResult=leftPhotonCamera.getLatestResult();
      this.validTarget=leftResult.hasTargets();
      if (leftResult.hasTargets())
      {
     //TODO change this to find specific target
      this.target = leftResult.getBestTarget();
      Pose2d leftEst=getEstimatedGlobalPose(leftPhotonPoseEstimator).get().estimatedPose.toPose2d();
      if (!(leftEst==null)){
      double x =leftEst.getX();
      double y =leftEst.getY();
      double Rot =leftEst.getRotation().getRadians();
      SmartDashboard.putNumber("pose X"+getName(),x);
      SmartDashboard.putNumber("pose Y"+getName(),y);
      SmartDashboard.putNumber("pose Rot"+getName(),Rot);
      }
      }
          var rightResult=leftPhotonCamera.getLatestResult();
      this.validTarget=rightResult.hasTargets();
      if (rightResult.hasTargets())
      {
     //TODO change this to find specific target
      this.target = rightResult.getBestTarget();
      Pose2d rightEst=getEstimatedGlobalPose(rightPhotonPoseEstimator).get().estimatedPose.toPose2d();
      if (!(rightEst==null)){
      double x =rightEst.getX();
      double y =rightEst.getY();
      double Rot =rightEst.getRotation().getRadians();
      SmartDashboard.putNumber("pose X right"+getName(),x);
      SmartDashboard.putNumber("pose Y right"+getName(),y);
      SmartDashboard.putNumber("pose Rot right "+getName(),Rot);
      }
      }
    }
    // This method will be called once per scheduler run
  }

