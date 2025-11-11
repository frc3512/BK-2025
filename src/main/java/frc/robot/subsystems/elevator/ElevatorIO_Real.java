package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoint;

public class ElevatorIO_Real implements ElevatorIO {

  private TalonFX motor;
  private MotionMagicVoltage setpoint = new MotionMagicVoltage(0); // Mechanism Rotations

  public ElevatorIO_Real() {

    // Motor Initialization
    motor = new TalonFX(ElevatorConstants.kMotorCANID);
    motor.getConfigurator().apply(ElevatorConstants.kMotorConfig);

    // Optimize Can Bus Utilization
    var motorVoltageSignal = motor.getMotorVoltage();
    var motorCurrentSignal = motor.getSupplyCurrent();
    var motorPositionSignal = motor.getPosition();
    var motorVelocitySignal = motor.getVelocity();
    var motorAccelerationSignal = motor.getAcceleration();

    motorVoltageSignal.setUpdateFrequency(50);
    motorCurrentSignal.setUpdateFrequency(50);
    motorPositionSignal.setUpdateFrequency(50);
    motorVelocitySignal.setUpdateFrequency(50);
    motorAccelerationSignal.setUpdateFrequency(50);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    motor.setControl(setpoint);

    // Update inputs
    inputs.motorVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrent = motor.getSupplyCurrent().getValueAsDouble();
    inputs.motorPosition =
        motor.getPosition().getValueAsDouble()
            * ElevatorConstants.kConversionFactor; // Convert rotations to inches
    inputs.motorVelocity =
        motor.getVelocity().getValueAsDouble()
            * ElevatorConstants.kConversionFactor; // Convert rps to ips
    inputs.motorAcceleration =
        motor.getAcceleration().getValueAsDouble()
            * ElevatorConstants.kConversionFactor; // Convert rps^2 to ips^2
    inputs.positionSetpoint =
        setpoint.Position * ElevatorConstants.kConversionFactor; // Convert rotations to inches
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
