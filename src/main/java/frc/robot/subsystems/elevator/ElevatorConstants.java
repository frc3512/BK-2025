// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;

/** Add your docs here. */
public class ElevatorConstants {

  // System Information

  public static final DCMotor kMotor = DCMotor.getFalcon500(2);
  public static final double kGearRatio = (5d / 1) * (66d / 22);

  public static final double kSprocketPD = 1.7567; // Get from Recalc
  public static final int kStages = 3; // Only needed for Cascade rigging.

  public static final double kConversionFactor =
      (Math.PI * kSprocketPD)
          * kStages; // Inches per Mechanism Revolution, remove stages for continuous rigging

  public static final double kMinHeight = 0.0; // Inches
  public static final double kMaxHeight = 60.0; // Inches
  public static final double kHeightTolerance = 1.0; // Inches

  // Motor Configuration Information

  public static final int kMotorCANID = 4;

  public static final TalonFXConfiguration kMotorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(kGearRatio))
          .withMotionMagic(
              new MotionMagicConfigs() // Example Values Only Needs Tuning
                  .withMotionMagicCruiseVelocity(100 / kConversionFactor) // 100 inches per second
                  .withMotionMagicAcceleration(
                      200 / kConversionFactor) // 200 inches per second squared
              )
          .withSlot0(new Slot0Configs().withKP(10)) // Example Values Only Needs Tuning
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(40)
                  .withStatorCurrentLimit(60)
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimitEnable(true));

  // Setpoints

  public static enum ElevatorSetpoint {
    kHome(0.0),
    kScore1(10.0),
    kScore2(60.0);

    public final double inches;

    private ElevatorSetpoint(double inches) {
      this.inches = inches;
    }
  }
}
