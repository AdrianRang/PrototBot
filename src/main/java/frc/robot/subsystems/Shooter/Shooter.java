package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
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
            .inverted(false);

        rightConfig
            .smartCurrentLimit(kCurrentLimit)
            .idleMode(IdleMode.kCoast)
            .openLoopRampRate(kRampRate)
            .inverted(true)
            .follow(leftMotor, true);

        this.leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        this.rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void set(double voltage) {
        this.leftMotor.setVoltage(voltage);
    }
    
    public void speedUp() {
        this.leftMotor.setVoltage(kShootVoltage);
    }

    public void stop() {
        this.leftMotor.stopMotor();
        this.rightMotor.stopMotor();
    }

    public boolean upToSpeed() {
        return this.leftMotor.getEncoder().getVelocity() > kMinShootSpeed;
    }

    public Command speedUpCommand() {
        return runOnce(this::speedUp);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
}
