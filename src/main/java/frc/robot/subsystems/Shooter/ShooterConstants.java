package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterConstants {
    public static final int kLeftMotorID = 31;
    public static final int kRightMotorID = 30;

    public static final double kShootVoltage = 6.5;
    public static final double kMinShootSpeed = 3400.0;

    public static final AngularVelocity kShootSpeed = RPM.of(3600);
    public static final double kEpsilon = 600;

    public static final double kV = 0.0018;
    public static final double kP = 0.00025;
    public static final double kI = 0;
    public static final double kD = 0.00001;

    public static final int kCurrentLimit = 30;
    public static final double kRampRate = 0.5;
}
