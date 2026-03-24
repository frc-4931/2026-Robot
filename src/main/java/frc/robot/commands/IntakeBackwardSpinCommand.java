package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeBackwardSpinCommand extends Command {
    private final IntakeSubsystem intake_subsystem;

    public IntakeBackwardSpinCommand(IntakeSubsystem subsystem) {
        intake_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        boolean success = intake_subsystem.IntakeSpin();
        if (!success) {
            intake_subsystem.ForwardSpin(-0.5);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
