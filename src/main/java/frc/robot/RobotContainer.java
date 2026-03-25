// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeBackwardSpinCommand;
import frc.robot.commands.IntakeGoToPositionCommand;
import frc.robot.commands.IntakeSpinCommand;
import frc.robot.commands.IntakeBackwardSpinCommand;
import frc.robot.commands.IntakeStopCommand;
import frc.robot.commands.LowerIntakeArm;
import frc.robot.commands.RaiseIntakeArm;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.StopIntakeCommand;
import frc.robot.commands.IntakeGoToPositionCommand;
import frc.robot.commands.IntakeGoToPositionZeroCommand;
import frc.robot.commands.IntakeGoToPositionNegPoint8Command;

import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.StopIntakeCommand;
import frc.robot.commands.SuperIntakeButton;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.ShooterMotorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import java.io.File;
import frc.robot.subsystems.IndexerSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandJoystick redbuttonbox= new CommandJoystick(1);
  final CommandJoystick otherbuttonbox= new CommandJoystick(2);
  // The robot's subsystems and commands are defined here...
  // private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/CompetitionChassis"));
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterMotorSubsystem shooterMotorSubsystem = new ShooterMotorSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();

  private final RaiseIntakeArm raiseIntakeCommand = new RaiseIntakeArm(intakeSubsystem);
  private final LowerIntakeArm lowerIntakeCommand = new LowerIntakeArm(intakeSubsystem);
  private final IntakeSpinCommand intakeSpinCommand = new IntakeSpinCommand(intakeSubsystem);
  private final IntakeBackwardSpinCommand intakeBackwardSpinCommand = new IntakeBackwardSpinCommand(intakeSubsystem);
  private final IntakeStopCommand intakeStopCommand = new IntakeStopCommand(intakeSubsystem);
  private final IntakeGoToPositionCommand intakeGoToPositionCommand = new IntakeGoToPositionCommand(intakeSubsystem);
  private final IntakeGoToPositionZeroCommand intakeGoToPositionZeroCommand = new IntakeGoToPositionZeroCommand(intakeSubsystem);
  private final IntakeGoToPositionNegPoint8Command intakeGoToPositionNegPoint8Command = new IntakeGoToPositionNegPoint8Command(intakeSubsystem);

  // private final Command runIntakeCommand = new ParallelCommandGroup(new LowerIntakeArm(intakeSubsystem), new IntakeSpinCommand(intakeSubsystem));
  // private final Command stowIntakeCommand = new ParallelCommandGroup(new RaiseIntakeArm(intakeSubsystem), new IntakeStopCommand(intakeSubsystem));
  private final RunIntakeCommand runIntakeCommand = new RunIntakeCommand(intakeSubsystem);
  private final StopIntakeCommand stopIntakeCommand = new StopIntakeCommand(intakeSubsystem);
  private final SuperIntakeButton superIntakeButtonCommandJoystick = new SuperIntakeButton(runIntakeCommand, stopIntakeCommand);

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;
  

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("indexer_spin", indexer.BackwardSpin());
    NamedCommands.registerCommand("shooter_spin", shooterMotorSubsystem.SpinAtSpeed(.75));
    NamedCommands.registerCommand("lower_arm", intakeSubsystem.goToPositionCommand(26.5));
    NamedCommands.registerCommand("run_intake_roller", intakeBackwardSpinCommand);

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));
    
    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

    }

    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());

      driverXbox.a().onTrue(lowerIntakeCommand);
      driverXbox.b().onTrue(raiseIntakeCommand);
      driverXbox.x().whileTrue(intakeSpinCommand);
      driverXbox.y().whileTrue(intakeStopCommand);
      driverXbox.rightTrigger().onTrue(shooterMotorSubsystem.SpinAtSpeed(0.5));
      driverXbox.leftTrigger().whileTrue(
          shooterMotorSubsystem.sysIdQuasistatic(Direction.kForward));

      driverXbox.povUp().whileTrue(
          shooterMotorSubsystem.sysIdQuasistatic(Direction.kForward)
          .onlyIf(DriverStation::isTest)
          );
      driverXbox.povDown().whileTrue(
          shooterMotorSubsystem.sysIdQuasistatic(Direction.kReverse)
          .onlyIf(DriverStation::isTest)
          );
      driverXbox.povRight().whileTrue(
          shooterMotorSubsystem.sysIdDynamic(Direction.kForward)
          .onlyIf(DriverStation::isTest)
          );
      driverXbox.povLeft().whileTrue(
          shooterMotorSubsystem.sysIdDynamic(Direction.kReverse)
          .onlyIf(DriverStation::isTest)
          );
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.y().onTrue(superIntakeButtonCommandJoystick);
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().whileTrue(intakeGoToPositionCommand);
      redbuttonbox.button(1).onTrue(indexer.BackwardSpin());
      redbuttonbox.button(2).whileTrue(indexer.ForwordSpin());
      redbuttonbox.button(3).whileTrue(intakeSpinCommand);
      redbuttonbox.button(4).whileTrue(intakeBackwardSpinCommand);
      redbuttonbox.button(5).whileTrue(intakeStopCommand);
      redbuttonbox.button(6).whileTrue(intakeGoToPositionZeroCommand);
      redbuttonbox.button(7).whileTrue(intakeGoToPositionNegPoint8Command);
    //TODO: ask Eddie how to fix these.
      redbuttonbox.button(8).whileTrue(intakeSubsystem.RunIntake());
      // Why is this subsystem function not returning a runable? check the subsystem again.
      redbuttonbox.button(9).whileTrue(raiseIntakeCommand);
      redbuttonbox.button(10).whileTrue(shooterMotorSubsystem.SpinAtSpeed(0.75));
      otherbuttonbox.button(1).whileTrue(shooterMotorSubsystem.SpinAtSpeed(-0.5));
      otherbuttonbox.button(2).whileTrue(shooterMotorSubsystem.SpinStop());
      otherbuttonbox.button(3).whileTrue(shooterMotorSubsystem.SpinAtSpeed(.9));
      otherbuttonbox.button(4).whileTrue(indexer.StopSpin());
      otherbuttonbox.button(5).whileTrue(indexer.BackwardSlowSpin());
      

      driverXbox.b().whileTrue(indexer.StopSpin());
      driverXbox.rightTrigger().whileTrue(intakeSubsystem.goToPositionCommand(0));
      driverXbox.leftTrigger().whileTrue(intakeSubsystem.goToPositionCommand(26.5));
     

      // buttonBox2.(button5 ).onTrue(algaeArm.ArmStop().andThen(roller.CoralStop()).andThen(climber.ClimbStop()));


    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
