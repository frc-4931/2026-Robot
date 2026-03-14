// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/*
 * TODO:
 * 
 * Exicute a comand to have the motor rpm fuctuate in a sine patern.
 */

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RaiseIntakeArm extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  // private final ExampleSubsystem m_subsystem;
  private final IntakeSubsystem intake_subsystem;
  private double index = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RaiseIntakeArm(IntakeSubsystem subsystem) {
    intake_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // intake_subsystem.armRaise();
    // intake_subsystem.stopRoller();
    // double speed = 400 * Math.sin(index*Math.PI);
    // System.out.println(speed);
    // intake_subsystem.runSecondMotor(speed);
    // index += 0.001;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
