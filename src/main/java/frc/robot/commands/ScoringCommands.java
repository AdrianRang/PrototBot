package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Intake.Intake;

public class ScoringCommands {
    public static Command shootCommand(Shooter shooter, Indexer indexer, Intake intake) {
        return shooter.speedUpCommand()
            .until(shooter::upToSpeed)
            .andThen(indexer.passCommand().alongWith(intake.intakeCommand()))
            .finallyDo((boolean interrupted) -> {
                intake.stop();
                shooter.stop();
                indexer.stop();
            }
        );
    }

    public static Command continuousShootCommand(Shooter shooter, Indexer indexer) {
        return new RunCommand(() -> {
            shooter.speedUp();
            if (shooter.upToSpeed()) {
                indexer.pass();
            } else {
                indexer.stop();
            }
        }) .finallyDo((boolean interrupted) -> {
            shooter.stop();
            indexer.stop();
        });
    }

    public static Command stop(Shooter shooter, Indexer indexer) {
        return shooter.stopCommand().alongWith(indexer.stopCommand());
    }

    public static Command ejectIntake(Intake intake, Indexer indexer) {
        return (intake.ejectCommand().alongWith(indexer.hopperEjectCommand()));
    }
}
