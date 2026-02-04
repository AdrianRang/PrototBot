package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.Shooter;

public class ScoringCommands {
    public static Command shootCommand(Shooter shooter, Indexer indexer) {
        return shooter.speedUpCommand()
            .until(shooter::upToSpeed)
            .andThen(indexer.passCommand())
            .finallyDo(() -> {
                shooter.stop();
                indexer.stopCommand();
            }
            );
    }
}
