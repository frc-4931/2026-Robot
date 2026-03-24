package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeGoToPositionNegPoint8Command extends Command {
    private final IntakeSubsystem intake_subsystem;

    public IntakeGoToPositionNegPoint8Command(IntakeSubsystem subsystem) {
        intake_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        intake_subsystem.goToPositionCommand(-0.8);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
