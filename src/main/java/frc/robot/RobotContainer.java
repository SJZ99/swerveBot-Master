// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_Joystick = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    m_driverController.getLeftY() *0.4,
                    m_driverController.getLeftX()*0.4,
                    m_driverController.getRightX() *0.8,
                    true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
  
    new JoystickButton(m_Joystick, 4).onTrue(new InstantCommand(() -> m_robotDrive.drive( 0,0,0.8,false), m_robotDrive));

    new JoystickButton(m_Joystick, 3).onTrue(Commands.run(m_robotDrive::level).until(null));
   
  

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
public Command getAutonomousCommand() { 


    String trajectoryJSON = "output/Path1.wpilib.json";
    Trajectory trajectory = new Trajectory();
    
    
        try {
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
          DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
          

      var thetaController =
      new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
  thetaController.enableContinuousInput(-Math.PI, Math.PI);


    var controller = new HolonomicDriveController(
      new PIDController(0.05, 0, 0), new PIDController(0.05, 0, 0),
      new ProfiledPIDController(0.05, 0, 0,
        new TrapezoidProfile.Constraints(6.28, 3.14)));

// Reset odometry to the starting pose of the trajectory.

m_robotDrive.resetOdometry(trajectory.getInitialPose());

// Run path following command, then stop at the end.

// Sample the trajectory at 3.4 seconds from the beginning.
Trajectory.State goal = trajectory.sample(3.4);

// Get the adjusted speeds. Here, we want the robot to be facing
// 70 degrees (in the field-relative coordinate system).
ChassisSpeeds adjustedSpeeds = controller.calculate(
  m_robotDrive.getPose(), goal, Rotation2d.fromDegrees(70.0));



  SwerveControllerCommand swerveControllerCommand =
      new SwerveControllerCommand(
        trajectory,
          m_robotDrive::getPose, // Functional interface to feed supplier
          DriveConstants.kDriveKinematics,

          // Position controllers
          new PIDController(AutoConstants.kPXController, 0, 0),
          new PIDController(AutoConstants.kPYController, 0, 0),
          thetaController,
          m_robotDrive::setModuleStates,
          m_robotDrive);

  // Reset odometry to the starting pose of the trajectory.
  m_robotDrive.resetOdometry(trajectory.getInitialPose());

  //Run path following command, then stop at the end.
  return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, true));

  }

 //Create config for trajectory
  //   TrajectoryConfig config =
  //       new TrajectoryConfig(
  //               AutoConstants.kMaxSpeedMetersPerSecond,
  //               AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  //           // Add kinematics to ensure max speed is actually obeyed
  //           .setKinematics(DriveConstants.kDriveKinematics);

  //  // An example trajectory to follow.  All units in meters.
  //   Trajectory exampleTrajectory =
  //       TrajectoryGenerator.generateTrajectory(
  //           // Start at the origin facing the +X direction
  //           new Pose2d(0, 0, new Rotation2d(0)),
  //           // Pass through these two interior waypoints, making an 's' curve path
  //           List.of(),
  //           // End 3 meters straight ahead of where we started, facing forward
  //           new Pose2d(3, 0, new Rotation2d(0)),
  //           config);

  //   var thetaController =
  //       new ProfiledPIDController(
  //           AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

  //   SwerveControllerCommand swerveControllerCommand =
  //       new SwerveControllerCommand(
  //           exampleTrajectory,
  //           m_robotDrive::getPose, // Functional interface to feed supplier
  //           DriveConstants.kDriveKinematics,

  //           // Position controllers
  //           new PIDController(AutoConstants.kPXController, 0, 0),
  //           new PIDController(AutoConstants.kPYController, 0, 0),
  //           thetaController,
  //           m_robotDrive::setModuleStates,
  //           m_robotDrive);

  //   // Reset odometry to the starting pose of the trajectory.
  //   m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

  //   //Run path following command, then stop at the end.
  //   return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
   
  // }

 }
