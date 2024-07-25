// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonvisionTurret;
import frc.robot.subsystems.Shooter;

public class AutoAimAtTag extends Command {
  private double yaw;
  private PhotonvisionTurret sight;
  private PhotonTrackedTarget target=null;
  private Shooter shooter;

  /** Creates a new AutoAimAtTag. */
  public AutoAimAtTag(PhotonvisionTurret sight,Shooter kshooter) {
    this.sight=sight;
    this.shooter=kshooter;
    addRequirements(sight);
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     target = sight.lookForTag(4);
    if(target!=null){
       this.yaw = -target.getYaw();
      }
      else{
        this.yaw=0;
      }
    shooter.autoAimTurret(yaw);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
