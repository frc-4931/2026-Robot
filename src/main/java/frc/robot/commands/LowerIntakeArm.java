/*
 *  
 * Execute the LowerIntakeArm command to lower the intake arm. This command will be triggered by a button press on the controller.
 */

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class LowerIntakeArm extends Command {
  private final IntakeSubsystem intake_subsystem;

  public LowerIntakeArm(IntakeSubsystem subsystem) {
    intake_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    boolean success = intake_subsystem.ArmLower();
    if (!success) {
      intake_subsystem.ArmLower();
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
