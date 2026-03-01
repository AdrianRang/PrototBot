package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Shooter extends SubsystemBase {
    private final SparkFlex leftMotor;
    private final SparkFlex rightMotor;

    public Shooter() {
        this.leftMotor = new SparkFlex(kLeftMotorID, MotorType.kBrushless);
        this.rightMotor = new SparkFlex(kRightMotorID, MotorType.kBrushless);

       SparkFlexConfig leftConfig = new SparkFlexConfig();
       SparkFlexConfig rightConfig = new SparkFlexConfig();

        leftConfig
            .smartCurrentLimit(kCurrentLimit)
            .idleMode(IdleMode.kCoast)
            .openLoopRampRate(kRampRate)
            .closedLoopRampRate(kRampRate)
            .inverted(false);

        leftConfig.closedLoop.feedForward.kV(kV);

        leftConfig.closedLoop
            .p(kP)
            .i(kI)
            .d(kD);

        rightConfig
            .smartCurrentLimit(kCurrentLimit)
            .idleMode(IdleMode.kCoast)
            .openLoopRampRate(kRampRate)
            .closedLoopRampRate(kRampRate)
            .inverted(false)
            .follow(leftMotor, true);

        this.leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        this.rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void set(double voltage) {
        this.leftMotor.setVoltage(voltage);
    }

    public void set(AngularVelocity velocity) {
        this.leftMotor.getClosedLoopController().setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    }
    
    public void speedUp() {
        // this.leftMotor.setVoltage(kShootVoltage);
        set(kShootSpeed);
    }

    public void stop() {
        this.leftMotor.stopMotor();
        this.rightMotor.stopMotor();
    }

    public boolean upToSpeed() {
        // return Math.abs(this.leftMotor.getEncoder().getVelocity()) > kMinShootSpeed;
        return Math.abs(this.leftMotor.getClosedLoopController().getSetpoint() - this.leftMotor.getEncoder().getVelocity()) < kEpsilon;
    }

    public Command speedUpCommand() {
        return run(this::speedUp);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter/UpToSpeed", upToSpeed());
        SmartDashboard.putNumber("Shooter/speed", this.leftMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/setpoint", this.leftMotor.getClosedLoopController().getSetpoint());
    }
 }
