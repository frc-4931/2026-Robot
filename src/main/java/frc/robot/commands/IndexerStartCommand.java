package frc.robot.commands;

import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IndexerStartCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;

    public IndexerStartCommand(IndexerSubsystem subsystem) {
        indexerSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
     indexerSubsystem.ForwordSpin();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
