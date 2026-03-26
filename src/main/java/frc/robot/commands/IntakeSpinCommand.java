package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSpinCommand extends Command {
    private final IntakeSubsystem intake_subsystem;

    public IntakeSpinCommand(IntakeSubsystem subsystem) {
        intake_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        boolean success = intake_subsystem.IntakeSpin();
        if (!success) {
            intake_subsystem.ForwardSpin(-IntakeConstants.INTAKE_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
