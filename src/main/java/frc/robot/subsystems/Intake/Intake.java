package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.subsystems.Intake.IntakeConstants.*;

public class Intake extends SubsystemBase {
  private final SparkMax motor;

  public Intake() {
    motor = new SparkMax(motorId, MotorType.kBrushless);

    SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(currentLimit)
      .inverted(true);

    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  public void set(double speed) {
    motor.set(speed);
  }

  public void intake() {
    set(intakeSpeed);
  }

  public void eject() {
    set(-intakeSpeed);
  }

  public void stop() {
    motor.stopMotor();
  }

  public Command intakeCommand() {
    return runOnce(this::intake);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public Command ejectCommand() {
    return runOnce(this::eject);
  }
}
