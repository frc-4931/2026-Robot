package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

    private final SparkMax indexerMotor;
    /**
     * This subsytem that controls the motor.
     */
    public IndexerSubsystem () {

        // Set up the motor as a brushed motor
        indexerMotor = new SparkMax(IndexerConstants.SINGLE_MOTOR_ID, MotorType.kBrushless);

        // Set can timeout. Because this project only sets parameters once on
        // construction, the timeout can be long without blocking robot operation. Code
        // which sets or gets parameters during operation may need a shorter timeout.
        indexerMotor.setCANTimeout(250);

        // Create and apply configuration for motor. Voltage compensation helps
        // the motor behave the same as the battery
        // voltage dips. The current limit helps prevent breaker trips or burning out
        // the motor in the event the practice stalls.
        SparkMaxConfig practiceConfig = new SparkMaxConfig();
        practiceConfig.voltageCompensation(IndexerConstants.SINGLE_MOTOR_VOLTAGE_COMP);
        practiceConfig.smartCurrentLimit(IndexerConstants.SINGLE_MOTOR_CURRENT_LIMIT);
        // practiceConfig.idleMode(IdleMode.kBrake);
        practiceConfig.idleMode(IdleMode.kCoast);
        indexerMotor.configure(practiceConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    }

    @Override
    public void periodic() {
    }

    /**
     *  This is a method that makes the practice spin to your desired speed.
     *  Positive values make it spin forward and negative values spin it in reverse.
     * 
     * @param speedmotor speed from -1.0 to 1, with 0 stopping it
     */
    public void runDutyCycleMotor(double speed){
        indexerMotor.set(speed);
    }

    // Change your command methods to use runEnd
    public Command ForwordSpin() {
        return this.runOnce(
            () -> runDutyCycleMotor(IndexerConstants.SINGLE_MOTOR_SPEED) // Run at 50% speed while held
        );
    }

    public Command BackwardSpin() {
        return this.runOnce(
            () -> runDutyCycleMotor(-IndexerConstants.SINGLE_MOTOR_SPEED)// Run reverse while held
        );
    }

    // public Command BackwardSlowSpin() {
    //     return this.runEnd(
    //         () -> runDutyCycleMotor(-0.2),
    //         () -> runDutyCycleMotor(0.0)
    //     );
    // }

    // The run once will have the motor continue to run until commanded to stop
    public Command ContinuousSlowForwardSpin() {
        return this.runOnce(
            () -> runDutyCycleMotor(-0.2)
        );
    }

}