package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Debug command to identify all CAN devices and their IDs on the bus.
 * Run this to verify encoder and motor assignments.
 */
public class CANBusDebugCommand extends Command {
  
  public CANBusDebugCommand() {
    setName("CAN Bus Debug");
  }

  @Override
  public void initialize() {
    System.out.println("\n========== CAN BUS DEVICE SCAN ==========");
    System.out.println("Scanning for devices...\n");
    
    scanCANcoders();
    scanSparkMotors();
    
    System.out.println("========== END SCAN ==========\n");
  }

  private void scanCANcoders() {
    System.out.println("--- Absolute Encoders (ThriftyEncoders) ---");
    for (int id = 0; id < 16; id++) {
      try {
        CANcoder encoder = new CANcoder(id);
        double position = encoder.getAbsolutePosition().getValueAsDouble() * 360;
        System.out.printf("CANcoder ID %d: Found! Position: %.2f°\n", id, position);
        SmartDashboard.putNumber("CANcoder_" + id + "_Position", position);
      } catch (Exception e) {
        // Device not found, continue
      }
    }
  }

  private void scanSparkMotors() {
    System.out.println("\n--- SparkMax/SparkFlex Motors ---");
    System.out.println("Checking IDs 0-30 for active motors...");
    
    for (int id = 0; id <= 30; id++) {
      // Note: These will show if they're responding to the bus
      // You may need to check firmware/CAN status for real status
      try {
        SparkMax sparkmax = new SparkMax(id, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        System.out.printf("Motor ID %d: Detected (SparkMax)\n", id);
        SmartDashboard.putBoolean("Motor_" + id + "_Found", true);
      } catch (Exception e) {
        // Continue
      }
    }
  }

  @Override
  public void execute() {
    // Command runs once and completes
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
