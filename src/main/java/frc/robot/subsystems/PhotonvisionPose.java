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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;


public class PhotonvisionPose extends SubsystemBase {
  private PhotonCamera leftPhotonCamera=new PhotonCamera(VisionConstants.CameraOne);
  private PhotonCamera rightPhotonCamera=new PhotonCamera(VisionConstants.CameraTwo);
  private AprilTagFieldLayout field=VisionConstants.field;
  private Transform3d leftcameraToRobot=VisionConstants.LeftCamtobot;
  private Transform3d rightcameraToRobot=VisionConstants.RightCamtobot; //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  private PhotonPoseEstimator leftPhotonPoseEstimator = new PhotonPoseEstimator(this.field, PoseStrategy.LOWEST_AMBIGUITY,this.leftPhotonCamera,this.leftcameraToRobot);
  private PhotonPoseEstimator rightPhotonPoseEstimator = new PhotonPoseEstimator(this.field, PoseStrategy.LOWEST_AMBIGUITY,this.rightPhotonCamera,this.rightcameraToRobot);


  private PhotonTrackedTarget target;
  private boolean validTarget;
  private EstimatedRobotPose leftEst;

  private EstimatedRobotPose rightEst;

  
  
  /** Creates a new Photonvision. */
  public PhotonvisionPose() 
  {
   
    
  
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

public EstimatedRobotPose getLeftEstimatedRobotPose(){
  return leftEst;
}
public EstimatedRobotPose getRightEstimatedRobotPose(){
  return rightEst;
}


  @Override
  public void periodic() {
    
      
      // if not find the best target and return it
      var leftResult=leftPhotonCamera.getLatestResult();
      this.validTarget=leftResult.hasTargets();
      if (leftResult.hasTargets())
      {
     
      this.target = leftResult.getBestTarget();
      var left=getEstimatedGlobalPose(leftPhotonPoseEstimator);
      if (left.isPresent()){
      leftEst= left.get();
      

      }
      }
          var rightResult=rightPhotonCamera.getLatestResult();
      this.validTarget=rightResult.hasTargets();
      if (rightResult.hasTargets())
      {
   
      this.target = rightResult.getBestTarget();
      var right=getEstimatedGlobalPose(rightPhotonPoseEstimator);
      if (right.isPresent()){
      rightEst=right.get();
      
      }
      }
    }
    // This method will be called once per scheduler run
  }

