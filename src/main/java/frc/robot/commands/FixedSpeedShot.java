package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ShooterSubsystem;

public class FixedSpeedShot extends Command {
  private final ShooterSubsystem secondMotor;
  private final double targetRPM;
  private final Timer timer = new Timer();

  public FixedSpeedShot(ShooterSubsystem secondMotor, double rpm) {
    this.secondMotor = secondMotor;
    this.targetRPM = rpm;
    addRequirements(secondMotor);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    secondMotor.setVelocity(targetRPM);
  }

  @Override
  public boolean isFinished() {
    // Command ends when timer exceeds 5 seconds
    return timer.hasElapsed(5.0);
  }

  @Override
  public void end(boolean interrupted) {
    secondMotor.stop();
    timer.stop();
  }
}