package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkMaxLimitSwitch;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.math.jni.ArmFeedforwardJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

// NEW: Imports for SysId and Units
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax armLeaderMotor;
    private final SparkMax armFollowerMotor; 
    private final SparkFlex intakeRollerMotor;
    private final SparkClosedLoopController pidController;
    private final SparkLimitSwitch forwardLimitSwitch;
    private final SparkLimitSwitch reverseLimitSwitch;
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

        armLeaderConfig.closedLoop.allowedClosedLoopError(0.04, ClosedLoopSlot.kSlot0);
        // 3.5 Configure magnetic limit switches on leader
        armLeaderConfig.limitSwitch
            .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
            .forwardLimitSwitchEnabled(true); // Automatically stops motor when hit
            // .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
            // .reverseLimitSwitchEnabled(true);
        this.forwardLimitSwitch = armLeaderMotor.getForwardLimitSwitch();
        this.reverseLimitSwitch = armLeaderMotor.getReverseLimitSwitch();

        armLeaderConfig.closedLoop
            .pid(.02, 0, 0.000, ClosedLoopSlot.kSlot0)
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

        intakeRollerMotor = new SparkFlex(IntakeConstants.INTAKE_ROLLER_MOTOR_ID, MotorType.kBrushless);

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
        // SmartDashboard.putNumber("armLeaderMotor", m_sysIdRoutine.);
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        if (isArmLimitHit()) {
            handleArmLimitHit();
        }
        pushvalue();
    }

    public Command SpinStop() {
        return this.runOnce(() -> { armLeaderMotor.stopMotor(); });
    }

    public Command ForwardSpin(double speed){
        return this.runOnce(() -> { armLeaderMotor.set(speed); });
    }

    public Command goToPositionCommand(double targetRotations) {
        SmartDashboard.putNumber("intakeArmSetpoint",targetRotations);
        // Keep continuously sending the target so our default "hold position" command doesn't
        // immediately override it.
        return run(() -> {
            pidController.setSetpoint(targetRotations, SparkMax.ControlType.kPosition);
        })
        .until(() -> Math.abs(armLeaderMotor.getEncoder().getPosition() - targetRotations) < 0.05);
    }

    private boolean isArmLimitHit() {
        return forwardLimitSwitch.isPressed() || reverseLimitSwitch.isPressed();
    }

    private void handleArmLimitHit() {
        armLeaderMotor.stopMotor();
        armFollowerMotor.stopMotor();
        armLeaderMotor.getEncoder().setPosition(0.0);
        armFollowerMotor.getEncoder().setPosition(0.0);
        SmartDashboard.putBoolean("armLimitHit", true);
        SmartDashboard.putNumber("armLeaderMotor", 0.0);
        SmartDashboard.putNumber("armFollowerMotor", 0.0);
    }

    private boolean moveArmTo(double targetRotations) {
        final double tolerance = 0.05;
        final double timeout = 2.0;
        final double startTime = Timer.getFPGATimestamp();
        pidController.setSetpoint(targetRotations, SparkMax.ControlType.kPosition);

        while (Timer.getFPGATimestamp() - startTime < timeout) {
            if (isArmLimitHit()) {
                handleArmLimitHit();
                if (targetRotations == IntakeConstants.ARM_LOWER) {
                    return true;
                }
                return false;
            }

            double error = Math.abs(armLeaderMotor.getEncoder().getPosition() - targetRotations);
            if (error < tolerance) {
                return true;
            }

            Timer.delay(0.01);
        }

        armLeaderMotor.stopMotor();
        return false;
    }

    public boolean ArmRaise() {
        boolean success = moveArmTo(IntakeConstants.ARM_RAISE);
        if (!success) {
            success = moveArmTo(IntakeConstants.ARM_RAISE);
        }
        return success;
    }

    public boolean ArmLower() {
        boolean success = moveArmTo(IntakeConstants.ARM_LOWER);
        if (!success) {
            success = moveArmTo(IntakeConstants.ARM_LOWER);
        }
        return success;
    }

    public boolean IntakeSpin() {
        try {
            intakeRollerMotor.set(IntakeConstants.INTAKE_SPEED);
            // verify motor current output not 0 (best-effort check)
            return Math.abs(intakeRollerMotor.get()) > 0.01;
        } catch (Exception e) {
            return false;
        }
    }

    public boolean IntakeBackwardsSpin() {
        try {
            intakeRollerMotor.set(-IntakeConstants.INTAKE_SPEED);
            // verify motor current output not 0 (best-effort check)
            return Math.abs(intakeRollerMotor.get()) > 0.01;
        } catch (Exception e) {
            return false;
        }
    }
    
    public boolean IntakeStop() {
        try {
            intakeRollerMotor.stopMotor();
            return Math.abs(intakeRollerMotor.get()) < 0.01;
        } catch (Exception e) {
            return false;
        }
    }

    public Command RunIntake() {
        return this.runOnce(() -> {
            boolean armOk = ArmLower();
            if (!armOk) {
                armOk = ArmLower();
            }
            boolean rollerOk = IntakeSpin();
            if (!rollerOk) {
                rollerOk = IntakeSpin();
            }
            if (!armOk || !rollerOk) {
                System.out.println("RunIntake: failed first attempt; armOk=" + armOk + ", rollerOk=" + rollerOk);
            }
        });
    }

    public Command StoweIntake() {
        return this.runOnce(() -> {
            boolean armOk = ArmRaise();
            if (!armOk) {
                armOk = ArmRaise();
            }
            boolean stopOk = IntakeStop();
            if (!stopOk) {
                stopOk = IntakeStop();
            }
            if (!armOk || !stopOk) {
                System.out.println("StoweIntake: failed first attempt; armOk=" + armOk + ", stopOk=" + stopOk);
            }
        });
    }

    public void pushvalue() {
        SmartDashboard.putNumber("armLeaderMotor", armLeaderMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("armFollowerMotor", armFollowerMotor.getEncoder().getPosition());
    }

    public Command deployAndKeepSpinning(double targetRotations, double rollerSpeed) {
        return this.run(() -> {
            // 1. Continuously update the arm position
            pidController.setReference(targetRotations, SparkMax.ControlType.kPosition);
            
            // 2. Continuously spin the roller
            intakeRollerMotor.set(rollerSpeed);
        })
        // 3. This part defines when the ARM is "done", but we do NOT stop the roller here
        .until(() -> Math.abs(armLeaderMotor.getEncoder().getPosition() - targetRotations) < 0.1)
        // 4. Once the arm is in place, transition to a new command that just spins the roller forever
        .andThen(this.run(() -> intakeRollerMotor.set(rollerSpeed)));
    }

    public Command getIntakeToggleCommand(double targetRotations, double rollerSpeed) {
        return Commands.startEnd(
            // START: What to do when toggled ON
            () -> {
                pidController.setReference(targetRotations, SparkMax.ControlType.kPosition);
                intakeRollerMotor.set(rollerSpeed);
            },
            // END: What to do when toggled OFF (re-pressed)
            () -> {
                pidController.setReference(0, SparkMax.ControlType.kPosition);
                intakeRollerMotor.stopMotor();
            },
            this // Requirement
        );
    }


}
