package frc.robot.subsystems.Indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.Indexer.IndexerConstants.*;

public class Indexer extends SubsystemBase {
    private final SparkMax lowerMotor;
    private final SparkMax upperMotor;

    public Indexer() {
        this.lowerMotor = new SparkMax(kLowerID, MotorType.kBrushless);
        this.upperMotor = new SparkMax(kUpperID, MotorType.kBrushed);

        SparkMaxConfig lowerConfig = new SparkMaxConfig();
        SparkMaxConfig upperConfig = new SparkMaxConfig();

        lowerConfig
            .smartCurrentLimit(kCurrentLimit)
            .idleMode(IdleMode.kBrake)
            .openLoopRampRate(kRampRate)
            .inverted(false);

        upperConfig
            .smartCurrentLimit(kCurrentLimit)
            .idleMode(IdleMode.kBrake)
            .openLoopRampRate(kRampRate)
            .inverted(true);

        this.lowerMotor.configure(lowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        this.upperMotor.configure(upperConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void pass() {
        this.lowerMotor.setVoltage(kLowerPassSpeed);
        this.upperMotor.setVoltage(kUpperPassSpeed);
    }

    public void eject() {
        this.upperMotor.setVoltage(kUpperEjectSpeed);
    }

    public void stop() {
        this.upperMotor.stopMotor();
        this.lowerMotor.stopMotor();
    }

    public Command passCommand() {
        return runOnce(this::pass);
    }

    public Command ejectCommand() {
        return runOnce(this::eject);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
}
