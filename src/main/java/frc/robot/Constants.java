// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import lib.BlueShift.constants.CTRECANDevice;
import lib.BlueShift.constants.PIDFConstants;
import lib.BlueShift.constants.SwerveModuleOptions;
import lib.BlueShift.utils.SwerveChassis;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) 
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.05;
  }

  public static class SwerveChassisConstants {
    public static final CTRECANDevice kGyroDevice = new CTRECANDevice(34, "*");

    public static final class PhysicalModel {
      // * MAX DISPLACEMENT SPEED (and acceleration)
      public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(5.0);
      public static final LinearAcceleration kMaxAcceleration = MetersPerSecond.per(Second).of(5.0);
      public static final LinearAcceleration kMaxDeceleration = MetersPerSecond.per(Second).of(-5.0);

      // * MAX ROTATIONAL SPEED (and acceleration)
      public static final AngularVelocity kMaxAngularSpeed = DegreesPerSecond.of(270.0);
      public static final AngularAcceleration kMaxAngularAcceleration = DegreesPerSecond.per(Second).of(360.0);
      public static final AngularAcceleration kMaxAngularDeceleration = DegreesPerSecond.per(Second).of(-360.0);

      // * Slew rate limiters
      public static final SlewRateLimiter yLimiter = new SlewRateLimiter(
          Constants.SwerveChassisConstants.PhysicalModel.kMaxAcceleration.in(MetersPerSecondPerSecond),
          Constants.SwerveChassisConstants.PhysicalModel.kMaxDeceleration.in(MetersPerSecondPerSecond), 0);
      public static final SlewRateLimiter xLimiter = new SlewRateLimiter(
          Constants.SwerveChassisConstants.PhysicalModel.kMaxAcceleration.in(MetersPerSecondPerSecond),
          Constants.SwerveChassisConstants.PhysicalModel.kMaxDeceleration.in(MetersPerSecondPerSecond), 0);
      public static final SlewRateLimiter rotLimiter = new SlewRateLimiter(
          Constants.SwerveChassisConstants.PhysicalModel.kMaxAngularAcceleration.in(RadiansPerSecond.per(Second)),
          Constants.SwerveChassisConstants.PhysicalModel.kMaxAngularDeceleration.in(RadiansPerSecond.per(Second)), 0);

      // Drive wheel diameter
      public static final Distance kWheelDiameter = Inches.of(4);

      // Gear ratios
      public static final double kDriveMotorGearRatio = 1.0 / 6.12; // 6.12:1 Drive
      public static final double kTurningMotorGearRatio = 1.0 / 12.8; // 12.8:1 Steering

      // Conversion factors (Drive Motor)
      public static final double kDriveEncoder_RotationToMeter = kDriveMotorGearRatio * kWheelDiameter.in(Meters);
      public static final double kDriveEncoder_RPMToMeterPerSecond = kDriveEncoder_RotationToMeter / 60.0;

      // Conversion factors (Turning Motor)
      public static final double kTurningEncoder_Rotation = kTurningMotorGearRatio;
      public static final double kTurningEncoder_RPS = kTurningEncoder_Rotation / 60.0;

      // Robot Without bumpers measures
      public static final Distance kTrackWidth = Inches.of(26);
      public static final Distance kWheelBase = Inches.of(26);

      // Create a kinematics instance with the positions of the swerve modules
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
          SwerveChassis.sizeToModulePositions(kTrackWidth.in(Meters), kWheelBase.in(Meters)));

      // Path constraints
      public static final PathConstraints kPathConstraints = new PathConstraints(kMaxSpeed, kMaxAcceleration,
          kMaxAngularSpeed, kMaxAngularAcceleration);
    }

    public static final class AutonomousConstants {
      public static final PIDConstants kTranslatePIDConstants = new PIDConstants(4.0, 0.0, 0.0);
      public static final PIDConstants kRotatePIDConstants = new PIDConstants(3.5, 0.0, 0.0);
    }
  }

  // * Swerve modules configuration
  public static final class SwerveModuleConstants {
    // * Motor config
    // Ramp rates
    public static final double kDriveMotorRampRate = 0;
    public static final double kTurningMotorRampRate = 0;

    // Current limits
    public static final int kDriveMotorCurrentLimit = 40;
    public static final int kDriveMotorLowerCurrentLimit = 30;
    public static final int kTurningMotorCurrentLimit = 30;

    // * PID
    public static final PIDFConstants kTurningPIDConstants = new PIDFConstants(1.57);

    // * Swerve modules options
    public static final SwerveModuleOptions kFrontLeftOptions = new SwerveModuleOptions()
        .setDriveMotorID(2)
        .setTurningMotorID(3)
        .setAbsoluteEncoderCANDevice(new CTRECANDevice(4, "*"))
        .setName("Front Left");

    public static final SwerveModuleOptions kFrontRightOptions = new SwerveModuleOptions()
        .setDriveMotorID(5)
        .setTurningMotorID(6)
        .setAbsoluteEncoderCANDevice(new CTRECANDevice(7, "*"))
        .setName("Front Right");

    public static final SwerveModuleOptions kBackLeftOptions = new SwerveModuleOptions()
        .setDriveMotorID(8)
        .setTurningMotorID(9)
        .setAbsoluteEncoderCANDevice(new CTRECANDevice(10, "*"))
        .setName("Back Left");

    public static final SwerveModuleOptions kBackRightOptions = new SwerveModuleOptions()
        .setDriveMotorID(11)
        .setTurningMotorID(12)
        .setAbsoluteEncoderCANDevice(new CTRECANDevice(13, "*"))
        .setName("Back Right");
  }

  public static final double startupStatusSignalTimeout = 20;
  public static final double deviceCheckPeriod = 5;
}
