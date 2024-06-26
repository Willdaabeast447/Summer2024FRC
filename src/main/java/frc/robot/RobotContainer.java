// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.XboxController;
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
  /*/  // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));*/
    
      return new PathPlannerAuto("Example Auto");
    
  }
}
