/*
 *  
 * Execute the LowerIntakeArm command to lower the intake arm. This command will be triggered by a button press on the controller.
 */

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SuperIntakeButton extends Command {
  private final Command runIntakeCommand;
  private final Command stopIntakeCommand;
  private boolean isRunning = true;

  public SuperIntakeButton(Command run, Command stop) {
    runIntakeCommand = run;
    stopIntakeCommand = stop;
    // intake_subsystem = subsystem;
    // addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if(isRunning){
      runIntakeCommand.initialize();
    }else{
      stopIntakeCommand.initialize();
    }
  }

  public void Execute(){
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(isRunning){
      isRunning = true;
    }else{
      isRunning = false;
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
