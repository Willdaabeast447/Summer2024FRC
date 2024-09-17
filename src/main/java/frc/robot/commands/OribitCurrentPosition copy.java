// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class OribitCurrentPosition extends Command {

   private DriveSubsystem drive;
  private DoubleSupplier joystickX;
  private DoubleSupplier joystickY;
  private DoubleSupplier rotspeed;
  private Pose2d goal;
  /** Creates a new OribitCurrentPosition. */
  public OribitCurrentPosition(DriveSubsystem drive, DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier rotspeed) {
    this.drive=drive;
    this.joystickX=joystickX;
    this.joystickY=joystickY;
    this.rotspeed=rotspeed;

    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.goal=drive.getPose();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double goalX=this.goal.getX();
    double goalY=this.goal.getY();
    Pose2d currenPose2d = drive.getPose();
    double currenPose2dX= currenPose2d.getX();
    double currenPose2dY=currenPose2d.getY();
    double deltaX=currenPose2dX-goalX;
    double deltaY=currenPose2dY-goalY;
    double dX2=deltaX*deltaX;
    double dY2=deltaY*deltaY;


    double r = Math.sqrt(dX2+dY2);
    double theta= Units.radiansToDegrees(Math.atan(deltaY/deltaX));
    if(deltaX<0){
      theta=theta+180;
    }
    else if (deltaY<0){
      theta=theta+360;
    }
    SmartDashboard.putNumber("R", r);
    SmartDashboard.putNumber("theta", theta);
    double tangettheta=(joystickX.getAsDouble()<=0)?theta+90:theta-90;
    tangettheta=tangettheta>360?tangettheta-360:tangettheta;
    SmartDashboard.putNumber("tantheta", tangettheta);
    int cosSign=1;
    int sinSign=1;
    if(theta<=90){
      if(joystickX.getAsDouble()<=0){
      cosSign=1;
      sinSign=1;
      }
      else
      {
      cosSign=-1;
      sinSign=-1;  
      }

    }
    else if (theta>90&&theta<=180){
        if(joystickX.getAsDouble()<=0){
      cosSign=1;
      sinSign=1;
      }
      else
      {
      cosSign=-1;
      sinSign=-1;  
      }
    }
    else if (theta>180&&theta<=270){
        if(joystickX.getAsDouble()<=0){
      cosSign=1;
      sinSign=1;
      }
      else
      {
      cosSign=-1;
      sinSign=-1;  
      }
    }
    else if (theta>270&&theta<=360){
        if(joystickX.getAsDouble()<=0){
      cosSign=1;
      sinSign=1;
      }
      else
      {
      cosSign=-1;
      sinSign=-1;  
      }
    }
    double rError=0.6-r;
    double rcorrection=rError*0.2;
    SmartDashboard.putNumber("r pid out", rcorrection);
    double Rvelx= -rcorrection*Math.cos(Units.degreesToRadians(theta));
    double Rvely= -rcorrection*Math.sin(Units.degreesToRadians(theta));

    double tangetVelX=(MathUtil.applyDeadband(joystickX.getAsDouble(),OIConstants.kDriveDeadband)*Math.cos(Units.degreesToRadians(tangettheta))*0.1*cosSign)+Rvelx;
    double tangetVelY=MathUtil.applyDeadband(joystickX.getAsDouble(),OIConstants.kDriveDeadband)*Math.sin(Units.degreesToRadians(tangettheta))*0.1*sinSign+Rvely;
    SmartDashboard.putNumber("tangetvely", tangetVelY);
    SmartDashboard.putNumber("tangetvelx", tangetVelX);
    
    if(r<0.5){
    drive.drive(
      MathUtil.applyDeadband(joystickY.getAsDouble(), OIConstants.kDriveDeadband),
      MathUtil.applyDeadband(joystickX.getAsDouble(), OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(rotspeed.getAsDouble(), OIConstants.kDriveDeadband),
      true, true);
    
    }
    else
    {
       drive.drive(tangetVelX,tangetVelY,-MathUtil.applyDeadband(rotspeed.getAsDouble(), OIConstants.kDriveDeadband), true,true );
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
