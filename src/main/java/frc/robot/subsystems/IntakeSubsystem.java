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
    private final SparkMax dutyCycleMotor;
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
        .pid(6.6472, 0, 0.000, ClosedLoopSlot.kSlot0)
        .feedForward
            .kS(0.14139)
            // .kV(0.12189)
            // .kA(a)
            // .kG(0.00099532) // kG is a linear gravity feedforward, for an elevator
            // .kS(0.14139, ClosedLoopSlot.kSlot0)
            // .kV(0.12189, ClosedLoopSlot.kSlot0)
            .kCos(0.00099532);
        pidController.setSetpoint(IntakeConstants.ARM_SETPOSITION,ControlType.kPosition);
        // armLeaderConfig.closedLoop
            // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // // The PID values (kP, kI, kD)
            // .pid(0.050423, 0, 0, ClosedLoopSlot.kSlot0)
            // // The Feedforward values (kS, kV) from SysId
            // .velocityFF(0.0) // Usually set to 0 when using kS/kV directly
            // .feedForward
            // .kS(0.0668, ClosedLoopSlot.kSlot0)
            // .kV(0.12175, ClosedLoopSlot.kSlot0);

        armLeaderMotor.configure(armLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

        dutyCycleMotor = new SparkMax(IntakeConstants.INTAKE_ROLLER_MOTOR_ID, MotorType.kBrushless);

        // Set can timeout. Because this project only sets parameters once on
        // construction, the timeout can be long without blocking robot operation. Code
        // which sets or gets parameters during operation may need a shorter timeout.
        dutyCycleMotor.setCANTimeout(250);

        // Create and apply configuration for motor. Voltage compensation helps
        // the motor behave the same as the battery
        // voltage dips. The current limit helps prevent breaker trips or burning out
        // the motor in the event the practice stalls.
        SparkMaxConfig practiceConfig = new SparkMaxConfig();
        practiceConfig.voltageCompensation(IntakeConstants.INTAKE_ROLLER_VOLTAGE_COMP);
        practiceConfig.smartCurrentLimit(IntakeConstants.INTAKE_ROLLER_CURRENT_LIMIT);
        // practiceConfig.idleMode(IdleMode.kBrake);
        practiceConfig.idleMode(IdleMode.kCoast);
        dutyCycleMotor.configure(practiceConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

    public void runSecondMotor(double speed){
        armLeaderMotor.set(speed);
    }

    public Command BackwardSlowSpin() {
        return this.runOnce(() -> { runSecondMotor(-.15);});
    }

    public Command SpinStop() {
        return this.runOnce(() -> { armLeaderMotor.stopMotor(); });
    }

    public void stop() {
        armLeaderMotor.stopMotor(); // Follower will also stop
    }

    public Command ForwordSlowSpin() {
        return this.runOnce(() -> { runSecondMotor(.15);});
    }

    public void setVelocity(double rpm) {
        // PID only needs to be sent to the leader; follower follows the output
        pidController.setReference(rpm, SparkBase.ControlType.kVelocity);
    }
    
}
