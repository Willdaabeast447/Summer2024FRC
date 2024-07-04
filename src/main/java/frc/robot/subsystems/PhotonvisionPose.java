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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
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
  private PhotonPoseEstimator leftPhotonPoseEstimator = new PhotonPoseEstimator(this.field, PoseStrategy.LOWEST_AMBIGUITY,this.leftPhotonCamera,this.leftcameraToRobot);
  private PhotonPoseEstimator rightPhotonPoseEstimator = new PhotonPoseEstimator(this.field, PoseStrategy.LOWEST_AMBIGUITY,this.rightPhotonCamera,this.rightcameraToRobot);
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
      var left=getEstimatedGlobalPose(leftPhotonPoseEstimator);
      if (!(left==null)){
      //Pose2d leftEst= left.get().estimatedPose.toPose2d();
      //double x =leftEst.getX();
      //double y =leftEst.getY();
      //double Rot =leftEst.getRotation().getRadians();
      //SmartDashboard.putNumber("pose X"+getName(),x);
      //SmartDashboard.putNumber("pose Y"+getName(),y);
      //SmartDashboard.putNumber("pose Rot"+getName(),Rot);
      }
      }
          var rightResult=rightPhotonCamera.getLatestResult();
      this.validTarget=rightResult.hasTargets();
      if (rightResult.hasTargets())
      {
     //TODO change this to find specific target
      this.target = rightResult.getBestTarget();
      var right=getEstimatedGlobalPose(rightPhotonPoseEstimator);
      if (!(right==null)){
      Pose3d rightEst=right.get().estimatedPose;
      //double x =rightEst.estimatedPose.getX();
      //double y =rightEst.getY();
      //double Rot =rightEst.getRotation().getRadians();
      //SmartDashboard.putData("EstimatedRobotPose", (Sendable) rightEst);
      //SmartDashboard.putNumber("pose Y right"+getName(),y);
      //SmartDashboard.putNumber("pose Rot right "+getName(),Rot);
      }
      }
    }
    // This method will be called once per scheduler run
  }

