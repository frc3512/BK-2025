// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoint;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public Command changeSetpoint(ElevatorSetpoint newSetpoint) {
    return runOnce(() -> io.changeSetpoint(newSetpoint));
  }

  @AutoLogOutput(key = "MechanismStates/ElevatorAtSetpoint")
  public boolean atSetpoint() {
    return Math.abs(inputs.positionSetpoint - inputs.motorPosition)
        < ElevatorConstants.kHeightTolerance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator/", inputs);
  }
}
