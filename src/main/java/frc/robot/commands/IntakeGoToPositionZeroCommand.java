package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeGoToPositionZeroCommand extends Command {
    private final IntakeSubsystem intake_subsystem;

    public IntakeGoToPositionZeroCommand(IntakeSubsystem subsystem) {
        intake_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        intake_subsystem.goToPositionCommand(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
