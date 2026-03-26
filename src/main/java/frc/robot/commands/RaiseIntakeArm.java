/*
 * 
 * Execute the RaiseIntakeArm command to raise the intake arm. This command will be triggered by a button press on the controller.
 */

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RaiseIntakeArm extends Command {
  private final IntakeSubsystem intake_subsystem;

  public RaiseIntakeArm(IntakeSubsystem subsystem) {
    intake_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    boolean success = intake_subsystem.ArmRaise();
    if (!success) {
      // retry once
      intake_subsystem.ArmRaise();
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
