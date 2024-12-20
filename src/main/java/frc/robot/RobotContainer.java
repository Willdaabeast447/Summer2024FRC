// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveWithPoseEstimation;
import frc.robot.commands.ManualShooter;
import frc.robot.commands.OribitCurrentPosition;
import frc.robot.commands.Shoot;
import frc.robot.commands.AutoAimAtBestTarget;
import frc.robot.commands.ContinuousAimAtTarget;
import frc.robot.commands.ShooterGetYaw;
import frc.robot.commands.SpinUP;
import frc.robot.commands.TestVisionPose;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Hoppper;
import frc.robot.subsystems.PhotonvisionPose;
import frc.robot.subsystems.PhotonvisionTurret;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Shooter m_Shooter=new Shooter();
  private final Hoppper hoppper=new Hoppper();
  private final PhotonvisionTurret sight= new PhotonvisionTurret(
    VisionConstants.CameraShooter,
    Constants.VisionConstants.field);
  private final PhotonvisionPose photonvisionPose=new PhotonvisionPose();

  /*private final PhotonvisionPose rightcam= new PhotonvisionPose(
    Constants.VisionConstants.CameraShooter,
    Constants.VisionConstants.field,
    VisionConstants.RightCamtobot);
  */

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  
  // Defining buttons for command triggers

  Trigger aButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  Trigger xButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  Trigger yButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  Trigger bButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  private SendableChooser<Command> autoChooser;  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    NamedCommands.registerCommand("SpinUP", new SpinUP(m_Shooter));
    NamedCommands.registerCommand("Shoot", new Shoot(m_Shooter, hoppper));
    // Configure the button bindings
    configureButtonBindings();


    m_robotDrive.setDefaultCommand(new DriveWithPoseEstimation(
          m_robotDrive,
          ()->m_driverController.getLeftY(),
          ()->m_driverController.getLeftX(), 
          ()->m_driverController.getRightX(),
           photonvisionPose));
           
    m_Shooter.setDefaultCommand(
      // hold the turret at pos 0 until auto aim is enabled
      new ManualShooter(hoppper,
                        m_Shooter,
                        ()->m_driverController.getRightBumper(),
                        ()->m_driverController.getLeftBumper(),
                        ()->m_driverController.getRightTriggerAxis())); 

    sight.setDefaultCommand(
      // report the yaw back to the dashboard 
      new ShooterGetYaw(sight));
    photonvisionPose.setDefaultCommand( new TestVisionPose(photonvisionPose));
    //rightcam.setDefaultCommand(new TestVisionPose(rightcam));
      
   
    autoChooser= AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("autochooser",autoChooser);
     
  }



  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
  
    
    xButton.toggleOnTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    yButton.toggleOnTrue(new OribitCurrentPosition(
            m_robotDrive,
            ()->m_driverController.getLeftX(),
            ()->m_driverController.getLeftY(),
            ()->m_driverController.getRightX()
            ));
    bButton.toggleOnTrue(new ContinuousAimAtTarget(sight, m_Shooter,m_robotDrive,4));
    aButton.toggleOnTrue(new AutoAimAtBestTarget(sight, m_Shooter));

   }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

      return this.autoChooser.getSelected();
    
  }
}
