package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoint;

public class ElevatorIO_Sim implements ElevatorIO {

  private TalonFX motor;
  private MotionMagicVoltage setpoint = new MotionMagicVoltage(0); // Mechanism Rotations

  private DCMotorSim motorModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              ElevatorConstants.kMotor, 0.001, ElevatorConstants.kGearRatio),
          ElevatorConstants.kMotor);

  public ElevatorIO_Sim() {

    // Motor Initialization
    motor = new TalonFX(ElevatorConstants.kMotorCANID);
    motor.getConfigurator().apply(ElevatorConstants.kMotorConfig);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    // Apply setpoints
    motor.setControl(setpoint);

    // Simulate motor
    var motorSim = motor.getSimState();
    motorSim.setSupplyVoltage(12);
    motorModel.setInputVoltage(motorSim.getMotorVoltage());
    motorModel.update(0.02);
    motorSim.setRawRotorPosition(
        motorModel.getAngularPosition().times(ElevatorConstants.kGearRatio));
    motorSim.setRotorVelocity(motorModel.getAngularVelocity().times(ElevatorConstants.kGearRatio));

    // Update inputs
    inputs.motorVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrent = motor.getSupplyCurrent().getValueAsDouble();
    inputs.motorPosition =
        motor.getPosition().getValueAsDouble()
            * ElevatorConstants.kConversionFactor; // Convert rotations to vertical inches
    inputs.motorVelocity =
        motor.getVelocity().getValueAsDouble()
            * ElevatorConstants.kConversionFactor; // Convert rps to inches per second
    inputs.motorAcceleration =
        motor.getAcceleration().getValueAsDouble()
            * ElevatorConstants.kConversionFactor; // Convert rps^2 to inches per second squared
    inputs.positionSetpoint =
        setpoint.Position
            * ElevatorConstants.kConversionFactor; // Convert rotations to vertical inches
  }

  @Override
  public void changeSetpoint(ElevatorSetpoint newSetpoint) {
    var inches =
        MathUtil.clamp(
            newSetpoint.inches, ElevatorConstants.kMinHeight, ElevatorConstants.kMaxHeight);
    setpoint.Position =
        inches / ElevatorConstants.kConversionFactor; // Convert degrees to rotations
  }
}
