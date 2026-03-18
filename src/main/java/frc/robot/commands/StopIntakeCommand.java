package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntakeCommand extends Command {
  private final IntakeSubsystem intake;

  public StopIntakeCommand(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    boolean armOk = intake.ArmRaise();
    if (!armOk) armOk = intake.ArmRaise();
    boolean rollerOk = intake.IntakeStop();
    if (!rollerOk) rollerOk = intake.IntakeStop();

    if (!armOk || !rollerOk) {
      System.out.println("StopIntake failed: arm=" + armOk + " roller=" + rollerOk);
    }
  }

  @Override
  public boolean isFinished() {
    return true; // one-shot
  }
}