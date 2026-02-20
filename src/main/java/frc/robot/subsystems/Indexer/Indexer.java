package frc.robot.subsystems.Indexer;

import static frc.robot.subsystems.Indexer.IndexerConstants.kCurrentLimit;
import static frc.robot.subsystems.Indexer.IndexerConstants.kHopperEjectSpeed;
import static frc.robot.subsystems.Indexer.IndexerConstants.kHopperId;
import static frc.robot.subsystems.Indexer.IndexerConstants.kHopperPassSpeed;
import static frc.robot.subsystems.Indexer.IndexerConstants.kLowerID;
import static frc.robot.subsystems.Indexer.IndexerConstants.kLowerPassSpeed;
import static frc.robot.subsystems.Indexer.IndexerConstants.kRampRate;
import static frc.robot.subsystems.Indexer.IndexerConstants.kUpperEjectSpeed;
import static frc.robot.subsystems.Indexer.IndexerConstants.kUpperID;
import static frc.robot.subsystems.Indexer.IndexerConstants.kUpperPassSpeed;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final SparkMax lowerMotor;
    private final SparkMax upperMotor;
    private final SparkFlex hopperMotor;

    public Indexer() {
        this.lowerMotor = new SparkMax(kLowerID, MotorType.kBrushless);
        this.upperMotor = new SparkMax(kUpperID, MotorType.kBrushed);
        this.hopperMotor = new SparkFlex(kHopperId, MotorType.kBrushless); 

        SparkMaxConfig lowerConfig = new SparkMaxConfig();
        SparkMaxConfig upperConfig = new SparkMaxConfig();
        SparkFlexConfig hopperConfig = new SparkFlexConfig();

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

        hopperConfig
            .smartCurrentLimit(kCurrentLimit)
            .idleMode(IdleMode.kBrake)
            .openLoopRampRate(kRampRate)
            .inverted(false);
            
        this.lowerMotor.configure(lowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        this.upperMotor.configure(upperConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        this.hopperMotor.configure(hopperConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void pass() {
        this.lowerMotor.setVoltage(kLowerPassSpeed);
        this.upperMotor.setVoltage(kUpperPassSpeed);
        this.hopperMotor.setVoltage(kHopperPassSpeed);
    }

    public void hopper() {
        this.hopperMotor.setVoltage(kHopperPassSpeed);
    }

    public void hopperEject() {
        this.hopperMotor.setVoltage(kHopperEjectSpeed);
    }

    public void eject() {
        this.upperMotor.setVoltage(kUpperEjectSpeed);
    }

    public void stop() {
        this.upperMotor.stopMotor();
        this.lowerMotor.stopMotor();
        this.hopperMotor.stopMotor();
    }

    public Command passCommand() {
        return run(this::pass);
    }

    public Command hopperCommand() {
        return run(this::hopper);
    }

    public Command hopperEjectCommand() {
        return run(this::hopperEject);
    }

    public Command ejectCommand() {
        return runOnce(this::eject);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
}
