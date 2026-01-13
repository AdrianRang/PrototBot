package frc.robot.speedAlterators;

import java.util.function.Supplier;

import org.opencv.core.Point;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static frc.robot.Constants.SwerveChassisConstants.AutonomousConstants.kRotatePIDConstants;
import lib.BlueShift.control.SpeedAlterator;

public class LookToward extends SpeedAlterator {
    private final Supplier<Pose2d> odoSupplier;
    private final Point target;
    private final PIDController pid;

    public LookToward(Supplier<Pose2d> odoSupplier, Point target) {
        this.target = target;
        this.odoSupplier = odoSupplier;

        this.pid = new PIDController(kRotatePIDConstants.kP, kRotatePIDConstants.kI, kRotatePIDConstants.kD);
    }

    @Override
    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        double angle = Math.atan2(odoSupplier.get().getY() - target.y, odoSupplier.get().getX() - target.x);
        double omegaRadiansPerSecond = pid.calculate(odoSupplier.get().getRotation().getRadians(), angle);

        return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omegaRadiansPerSecond);
    }
}
