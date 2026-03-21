package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterMotorConstants;

// NEW: Imports for SysId and Units
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class ShooterMotorSubsystem extends SubsystemBase {
    private final SparkMax leaderMotor;
    private final SparkMax followerMotor; 
    
    private final SparkClosedLoopController pidController;
    private final SysIdRoutine m_sysIdRoutine;

    public ShooterMotorSubsystem() {
        // 1. Initialize Motors
        leaderMotor = new SparkMax(ShooterMotorConstants.LEADER_ID, MotorType.kBrushless);
        followerMotor = new SparkMax(ShooterMotorConstants.FOLLOWER_ID, MotorType.kBrushless);
        
        pidController = leaderMotor.getClosedLoopController();

        // 2. Configure Leader
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.voltageCompensation(ShooterMotorConstants.VOLTAGE_COMP);
        leaderConfig.smartCurrentLimit(ShooterMotorConstants.CURRENT_LIMIT);
        leaderConfig.idleMode(IdleMode.kBrake);
        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // The PID values (kP, kI, kD)
            .pid(3.596, 0, 0, ClosedLoopSlot.kSlot0)
            // The Feedforward values (kS, kV) from SysId
            .velocityFF(0.0) // Usually set to 0 when using kS/kV directly
            .feedForward
            .kS(41.367, ClosedLoopSlot.kSlot0)
            .kV(4097.7, ClosedLoopSlot.kSlot0);

        leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // 3. Configure Follower to be Inverted
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.apply(leaderConfig); // Copy same limits/brake mode
        followerConfig.follow(leaderMotor, true); // FOLLOW and INVERT

        leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // 4. SysId Routine Setup
        m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                // Unified Drive: Sending voltage to leader automatically sends it to follower
                (voltage) -> leaderMotor.setVoltage(voltage.in(Volts)),
                log -> {
                    // We only need to log the Leader's encoder because they are physically linked
                    log.motor("dual-motor-system")
                       .voltage(Volts.of(leaderMotor.getAppliedOutput() * leaderMotor.getBusVoltage()))
                       .angularPosition(Rotations.of(leaderMotor.getEncoder().getPosition()))
                       .angularVelocity(RotationsPerSecond.of(leaderMotor.getEncoder().getVelocity() / 60.0));
                },
                this
            )
        );
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

    public void runLeadMotor(double speed){
        leaderMotor.set(speed);
    }

    public Command BackwardSlowSpin() {
        return this.runOnce(() -> { runLeadMotor(-.15);});
    }

    public Command SpinStop() {
        return this.runOnce(() -> { leaderMotor.stopMotor(); });
    }

    public void stop() {
        leaderMotor.stopMotor(); // Follower will also stop
    }

    public Command SpinAtSpeed(double speed) {
        return this.run(() -> { runLeadMotor(speed);});
    }

    public void setVelocity(double rpm) {
        // PID only needs to be sent to the leader; follower follows the output
        pidController.setReference(rpm, SparkBase.ControlType.kVelocity);
    }
}
