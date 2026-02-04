// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.SwerveChassisConstants;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.ScoringCommands;
import frc.robot.subsystems.SwerveChassis;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import lib.Elastic;
import lib.Elastic.Notification;
import lib.Elastic.Notification.NotificationLevel;
import lib.BlueShift.control.CustomController;
import lib.BlueShift.control.CustomController.CustomControllerType;
import lib.BlueShift.odometry.swerve.BlueShiftOdometry;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CustomController DRIVER = new CustomController(Constants.OperatorConstants.kDriverControllerPort, CustomControllerType.XBOX, Constants.OperatorConstants.kDeadband, 1);

  private final SwerveChassis chassis;
  private final Shooter shooter;
  private final Indexer indexer;

  private final BlueShiftOdometry m_odometry;

  private final SendableChooser<Command> m_autonomousChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    this.chassis = new SwerveChassis(
      new SwerveModule(Constants.SwerveModuleConstants.kFrontLeftOptions),
      new SwerveModule(Constants.SwerveModuleConstants.kFrontRightOptions),
      new SwerveModule(Constants.SwerveModuleConstants.kBackLeftOptions),
      new SwerveModule(Constants.SwerveModuleConstants.kBackRightOptions),
      new Gyro(new GyroIOPigeon(Constants.SwerveChassisConstants.kGyroDevice))
    );

    this.shooter = new Shooter();
    this.indexer = new Indexer();

    
    // this.m_limelight3G = new LimelightOdometryCamera("limelight_threeg", false, true, VisionOdometryFilters::visionFilter);
    
    this.m_odometry = new BlueShiftOdometry(
      Constants.SwerveChassisConstants.PhysicalModel.kDriveKinematics, 
      chassis::getHeading,
      chassis::getModulePositions,
      new Pose2d(),
      0.02
    );
      
    RobotConfig ppRobotConfig = null;
    try{
      ppRobotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "ERROR! COULD NOT LOAD PP ROBOT CONFIG", e.getMessage()));
      DriverStation.reportError("ERROR! COULD NOT LOAD PP ROBOT CONFIG", e.getStackTrace());
    }

    AutoBuilder.configure(
      m_odometry::getEstimatedPosition,
      m_odometry::resetPosition,
      chassis::getRobotRelativeChassisSpeeds,
      (ChassisSpeeds speeds, DriveFeedforwards ff) -> chassis.driveRobotRelative(speeds),
      new PPHolonomicDriveController(
        SwerveChassisConstants.AutonomousConstants.kTranslatePIDConstants,
        SwerveChassisConstants.AutonomousConstants.kRotatePIDConstants
      ),
      ppRobotConfig,
      () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
      chassis
    );

    this.m_autonomousChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("AutoChooser", m_autonomousChooser);

    SmartDashboard.putData("Chassis/ResetTurningEncoders", new InstantCommand(chassis::resetTurningEncoders).ignoringDisable(true));
    SmartDashboard.putData("Chassis/ZeroHeading", new InstantCommand(chassis::zeroHeading).ignoringDisable(true));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // While the left bumper on operator controller is held, intake Fuel
    // DRIVER.leftBumper()
    //     .whileTrue(ballSubsystem.spinUpCommand().finallyDo(ballSubsystem::stop));
    // // While the right bumper on the operator controller is held, spin up for 1
    // // second, then launch fuel. When the button is released, stop.
    // DRIVER.rightBumper()
    //     .whileTrue(ballSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
    //         .andThen(ballSubsystem.launchCommand())
    //         .finallyDo(() -> ballSubsystem.stop()));
    
    // // While the A button is held on the operator controller, eject fuel back out
    // // the intake
    // DRIVER.bottomButton()
    //     .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop()));
    
    // Trigger lookAtTrigger = new ToggleTrigger(DRIVER.rightButton());

    // lookAtTrigger.whileTrue(runEnd(() -> chassis.enableSpeedAlterator(new LookToward(m_odometry::getEstimatedPosition, new Point(4.62, 4.03))), () -> chassis.disableSpeedAlterator()));

    DRIVER.rightStickButton().onTrue(new InstantCommand(chassis::zeroHeading));

    DRIVER.rightButton().whileTrue(ScoringCommands.shootCommand(shooter, indexer));

    this.chassis.setDefaultCommand(new DriveSwerve(
        chassis,
        () -> -DRIVER.getLeftY(),
        () -> -DRIVER.getLeftX(),
        () -> DRIVER.getRightTrigger() - DRIVER.getLeftTrigger(),
        () -> !DRIVER.bottomButton().getAsBoolean()
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }
}
