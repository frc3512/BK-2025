// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeIntake;

import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaePivotSetpoint;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaeWheelsSetpoint;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface AlgaeIntakeIO {

  @AutoLog
  public static class AlgaeIntakeIOInputs {
    // algae pivot motor
    public double pivotMotorVoltage = 0.0; // Volts
    public double pivotMotorCurrent = 0.0; // Amps
    public double pivotMotorPosition = 0.0; // Degrees
    public double pivotMotorVelocity = 0.0; // Degrees per Second
    public double pivotMotorAcceleration = 0.0; // Degrees per Second Squared
    public double pivotPositionSetpoint = 0.0; // Degrees

    // algae wheels
    public double wheelsMotorVoltage = 0.0; // Volts
    public double wheelsMotorCurrent = 0.0; // Amps
    public double wheelsMotorTemp = 0.0; // Farenheight
  }

  public default void updateInputs(AlgaeIntakeIOInputs inputs) {}

  public default void changePivotSetpoint(AlgaePivotSetpoint newSetpoint) {}

  public default void changeWheelsSetpoint(AlgaeWheelsSetpoint newSetpoint) {}
}
