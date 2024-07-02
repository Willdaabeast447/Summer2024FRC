// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AutoAim;
import frc.robot.commands.ShooterGetYaw;
import frc.robot.commands.TestToggle;
import frc.robot.commands.drive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

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
  private final Photonvision sight= new Photonvision(
    VisionConstants.CameraShooter,
    Constants.VisionConstants.field,
    new Transform3d(0, 0, 0, null),
    false);
  private final Photonvision leftcam= new Photonvision(
    Constants.VisionConstants.CameraShooter,
    Constants.VisionConstants.field,
    VisionConstants.LeftCamtobot,
    true);
  private final Photonvision rightcam= new Photonvision(
    Constants.VisionConstants.CameraShooter,
    Constants.VisionConstants.field,
    VisionConstants.RightCamtobot,
    true);


  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  
  // Defining buttons for command triggers

  Trigger aButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  Trigger xButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  private SendableChooser<Command> autoChooser;  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new drive(m_robotDrive, 
        ()->m_driverController.getLeftY(),
        ()->m_driverController.getLeftX(), 
        ()->m_driverController.getRightX()));            
    m_Shooter.setDefaultCommand(
      // hold the turret at pos 0 until auto aim is enabled
      new TestToggle(m_Shooter, 0)); 

    sight.setDefaultCommand(
      // report the yaw back to the dashboard 
      new ShooterGetYaw(sight));
      
    /*
     * TODO ad default commands to  left and right cameras
     */
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
   /* new JoystickButton(m_driverController, aButton)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive)); */
    aButton.toggleOnTrue(new AutoAim(sight, m_Shooter));
    xButton.toggleOnTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

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
