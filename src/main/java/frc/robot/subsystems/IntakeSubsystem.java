package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.jni.ArmFeedforwardJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

// NEW: Imports for SysId and Units
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax armLeaderMotor;
    private final SparkMax armFollowerMotor; 
    private final SparkMax intakeRollerMotor;
    private final SparkClosedLoopController pidController;
    private final SysIdRoutine m_sysIdRoutine;

    public IntakeSubsystem() {
        // 1. Initialize Motors
        armLeaderMotor = new SparkMax(IntakeConstants.LEADER_ID, MotorType.kBrushless);
        armFollowerMotor = new SparkMax(IntakeConstants.FOLLOWER_ID, MotorType.kBrushless);
        
        pidController = armLeaderMotor.getClosedLoopController();

        // 2. Configure Leader
        SparkMaxConfig armLeaderConfig = new SparkMaxConfig();
        armLeaderConfig.voltageCompensation(IntakeConstants.VOLTAGE_COMP);
        armLeaderConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
        armLeaderConfig.idleMode(IdleMode.kBrake);

        armLeaderConfig.closedLoop
        .pid(15, 0, 0.000, ClosedLoopSlot.kSlot0)
        .feedForward
            .kS(0.14139)
            // .kV(0.12189)
            // .kA(a)
            // .kG(0.00099532) // kG is a linear gravity feedforward, for an elevator
            // .kS(0.14139, ClosedLoopSlot.kSlot0)
            // .kV(0.12189, ClosedLoopSlot.kSlot0)
            .kCos(0.00099532);

        // 3. Configure Follower to be Inverted
        SparkMaxConfig armFollowerConfig = new SparkMaxConfig();
        armFollowerConfig.apply(armLeaderConfig); // Copy same limits/brake mode
        armFollowerConfig.follow(armLeaderMotor, true); // FOLLOW and INVERT

        armLeaderMotor.configure(armLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armFollowerMotor.configure(armFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // 4. SysId Routine Setup
        m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                // Unified Drive: Sending voltage to leader automatically sends it to follower
                (voltage) -> armLeaderMotor.setVoltage(voltage.in(Volts)),
                log -> {
                    // We only need to log the Leader's encoder because they are physically linked
                    log.motor("dual-motor-system")
                       .voltage(Volts.of(armLeaderMotor.getAppliedOutput() * armLeaderMotor.getBusVoltage()))
                       .angularPosition(Rotations.of(armLeaderMotor.getEncoder().getPosition()))
                       .angularVelocity(RotationsPerSecond.of(armLeaderMotor.getEncoder().getVelocity() / 60.0));
                },
                this
            )
        );

        intakeRollerMotor = new SparkMax(IntakeConstants.INTAKE_ROLLER_MOTOR_ID, MotorType.kBrushless);

        // Set can timeout. Because this project only sets parameters once on
        // construction, the timeout can be long without blocking robot operation. Code
        // which sets or gets parameters during operation may need a shorter timeout.
        intakeRollerMotor.setCANTimeout(250);

        // Create and apply configuration for motor. Voltage compensation helps
        // the motor behave the same as the battery
        // voltage dips. The current limit helps prevent breaker trips or burning out
        // the motor in the event the practice stalls.
        SparkMaxConfig practiceConfig = new SparkMaxConfig();
        practiceConfig.voltageCompensation(IntakeConstants.INTAKE_ROLLER_VOLTAGE_COMP);
        practiceConfig.smartCurrentLimit(IntakeConstants.INTAKE_ROLLER_CURRENT_LIMIT);
        // practiceConfig.idleMode(IdleMode.kBrake);
        practiceConfig.idleMode(IdleMode.kCoast);
        intakeRollerMotor.configure(practiceConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // NEW: Command Factories to trigger tests from RobotContainer
    public Command sysIdQuasistatic(Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {}

    public Command SpinStop() {
        return this.runOnce(() -> { armLeaderMotor.stopMotor(); });
    }

    public Command goToPositionCommand(double targetRotations) {
        // Keep continuously sending the target so our default "hold position" command doesn't
        // immediately override it.
        return run(() -> {
            // Use kPosition for instant PID or kSmartMotion for a smooth profiled move
            pidController.setSetpoint(targetRotations, SparkMax.ControlType.kPosition);
        })
        // The command is finished when the encoder is within a small range of the target.
        // Use a tighter tolerance so we don't end early and immediately fall back to the
        // hold position command.
        .until(() -> Math.abs(armLeaderMotor.getEncoder().getPosition() - targetRotations) < 0.05);
    }


    public void ArmRaise() {
        goToPositionCommand(IntakeConstants.ARM_RAISE)
            .withTimeout(2) // Interupts if it takes longer than timeout seconds
            .handleInterrupt(() -> {
                // Optional: Logic to run if the command times out (e.g., stop motor)
                armLeaderMotor.set(0);
                System.out.println("Position command timed out - potential jam!");
            });
    }
    public boolean ArmLower(){
        goToPositionCommand(IntakeConstants.ARM_LOWER)
            .withTimeout(2) // Interupts if it takes longer than timeout seconds
            .handleInterrupt(() -> {
                // Optional: Logic to run if the command times out (e.g., stop motor)
                armLeaderMotor.set(0);
                System.out.println("Position command timed out - potential jam!");
            });
            return true;
    }

    public boolean IntakeSpin(){
        return true;

    }

    public boolean IntakeStop(){
        return true;
    }

    public void pushvalue() {
        SmartDashboard.putNumber("armLeaderMotor", armLeaderMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("armFollowerMotor", armFollowerMotor.getEncoder().getPosition());
    }

    public void setVelocity(double rpm) {
        // PID only needs to be sent to the leader; follower follows the output
        pidController.setSetpoint(rpm, SparkBase.ControlType.kVelocity);
    }
   
}
