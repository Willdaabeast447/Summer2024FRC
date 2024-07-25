// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PhotonvisionTurret extends SubsystemBase {
  PhotonCamera camera;
  AprilTagFieldLayout field;
  Transform3d cameraToRobot; //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  Optional<EstimatedRobotPose> estiPose3d;
   PhotonTrackedTarget target;
  private boolean validTargets;
  private PhotonPoseEstimator photonPoseEstimator;
  private PhotonPipelineResult result;
  
  
  /** Creates a new Photonvision. */
  public PhotonvisionTurret(String camerashooter, AprilTagFieldLayout field) 
  {
    this.field=field;
    this.camera=new PhotonCamera(camerashooter);
    
   
  }
 
  
  public int getTargetID(){
    return this.target.getFiducialId();
  }

  public double getTargetYaw(){
    SmartDashboard.putNumber("tag yaw",target.getYaw());
    return this.target.getYaw();
  }
  
  public  void getEstimatedGlobalPose() {
        
        estiPose3d= photonPoseEstimator.update();
         
    }

    public PhotonTrackedTarget lookForTag( int tagId)
    {
      PhotonTrackedTarget tempTarget=null;
      
       List<PhotonTrackedTarget> tagsFound=this.result.getTargets();
       if (this.validTargets){
      for (int i=0;i<tagsFound.size();i++) {
        
        if (tagsFound.get(i).getFiducialId()==tagId){
        tempTarget=tagsFound.get(i);
          break;
        }
       }
      }
       return tempTarget;
    }
 
    public PhotonTrackedTarget findBesTarget(){
      if(this.validTargets){
        return this.result.getBestTarget();
      }
      else{
        return null;
      }
    }

  @Override
  public void periodic() {    
      this.result=camera.getLatestResult();
      this.validTargets=result.hasTargets();
      /* getEstimatedGlobalPose();
       double x =estiPose3d.get().estimatedPose.getX();
       double y =estiPose3d.get().estimatedPose.getX();
       double Rot =estiPose3d.get().estimatedPose.getX();
       SmartDashboard.putNumber("pose X",x);
       SmartDashboard.putNumber("pose Y",x);
       SmartDashboard.putNumber("pose Rot",x);
       */
      }
    // This method will be called once per scheduler run
  }

