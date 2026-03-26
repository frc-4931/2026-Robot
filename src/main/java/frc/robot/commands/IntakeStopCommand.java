package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeStopCommand extends Command {
    private final IntakeSubsystem intake_subsystem;

    public IntakeStopCommand(IntakeSubsystem subsystem) {
        intake_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        boolean success = intake_subsystem.IntakeStop();
        if (!success) {
            intake_subsystem.SpinStop();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
