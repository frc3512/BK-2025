// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaePivotSetpoint;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaeWheelsSetpoint;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntake extends SubsystemBase {

  private AlgaeIntakeIO io;
  private AlgaeIntakeIOInputsAutoLogged inputs = new AlgaeIntakeIOInputsAutoLogged();

  public AlgaeIntake(AlgaeIntakeIO io) {
    this.io = io;
  }

  public Command changePivotSetpoint(AlgaePivotSetpoint newSetpoint) {
    return runOnce(() -> io.changePivotSetpoint(newSetpoint));
  }

  public Command changeWheelsSetpoint(AlgaeWheelsSetpoint newSetpoint) {
    return runOnce(() -> io.changeWheelsSetpoint(newSetpoint));
  }

  @AutoLogOutput(key = "MechanismStates/AlgaePivotAtSetpoint")
  public boolean atPivotSetpoint() {
    return Math.abs(inputs.pivotPositionSetpoint - inputs.pivotMotorPosition)
        < AlgaeIntakeConstants.kPivotAngleTolerance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AlgaeIntake/", inputs);
  }
}
