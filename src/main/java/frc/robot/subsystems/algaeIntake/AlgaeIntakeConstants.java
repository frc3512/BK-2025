// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeIntake;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
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
public class AlgaeIntakeConstants {

  // System Information

  // pivot motor
  public static final DCMotor kPivotMotor = DCMotor.getKrakenX60(1);
  public static final double kPivotGearRatio = (68d / 30) * (3d / 1);

  public static final double kPivotMinAngle = 0.0;
  public static final double kPivotMaxAngle = 90.0;
  public static final double kPivotAngleTolerance = 2.0;

  // wheels motor
  public static final DCMotor kWheelsMotor = DCMotor.getKrakenX60(1);
  public static final double kWheelsGearRatio = (24d / 12);

  // Motor Configuration Information
  public static final int kPivotMotorCANID = 16;
  public static final int kWheelsMotorCANID = 15;
  public static final int kPivotEncoderCANID = 30;

  public static final TalonFXConfiguration kPivotMotorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(kPivotGearRatio))
          .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(true))
          .withMotionMagic(
              new MotionMagicConfigs() // Example Values Only Needs Tuning
                  .withMotionMagicCruiseVelocity(0) // 360 degrees per second
                  .withMotionMagicAcceleration(0) // 1080 degrees per second square
              )
          .withSlot0(new Slot0Configs()
          .withKP(0)
          .withKG(0)
          .withKD(0)) // Example Values Only Needs Tuning
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(40)
                  .withStatorCurrentLimit(60)
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimitEnable(true));

  public static final TalonFXConfiguration kWheelsMotorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(20)
                  .withStatorCurrentLimit(30)
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimitEnable(true));

  // Setpoints

  public static enum AlgaePivotSetpoint {
    kHome(0.0),
    kIntake(45.0);

    public final double degrees;

    private AlgaePivotSetpoint(double degrees) {
      this.degrees = degrees;
    }
  }

  public static enum AlgaeWheelsSetpoint {
    kOff(0),
    kIntake(9),
    KIdle(2),
    kProcessor(-5);

    public final double volts;

    private AlgaeWheelsSetpoint(double volts) {
      this.volts = volts;
    }
  }
}
