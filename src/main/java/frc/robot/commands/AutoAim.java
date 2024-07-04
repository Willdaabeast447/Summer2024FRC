// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonvisionTurret;
import frc.robot.subsystems.Shooter;

public class AutoAim extends Command {
  private PhotonvisionTurret sight;
  private Shooter shooter;
  private double yaw=0;;

  /** Creates a new AutoAim. */
  public AutoAim(PhotonvisionTurret sight,Shooter kshooter) {
    this.sight=sight;
    this.shooter=kshooter;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(sight.hasTargets()){
       this.yaw = -sight.getTargetYaw();
      }
      else{
        this.yaw=0;
      }
    shooter.autoAimTurret(yaw);

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
