package frc.robot.subsystems.algaeIntake;

import static edu.wpi.first.units.Units.Fahrenheit;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaePivotSetpoint;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaeWheelsSetpoint;

public class AlgaeIntakeIO_Sim implements AlgaeIntakeIO {

  private TalonFX pivotMotor;
  private MotionMagicVoltage pivotSetpoint = new MotionMagicVoltage(0); // Mechanism Rotations

  private TalonFX wheelsMotor;
  private VoltageOut wheelsSetpoint = new VoltageOut(0); // Volts

  private DCMotorSim pivotMotorModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              AlgaeIntakeConstants.kPivotMotor, 0.001, AlgaeIntakeConstants.kPivotGearRatio),
          AlgaeIntakeConstants.kPivotMotor);

  private DCMotorSim wheelsMotorModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              AlgaeIntakeConstants.kWheelsMotor, 0.0001, AlgaeIntakeConstants.kWheelsGearRatio),
          AlgaeIntakeConstants.kWheelsMotor);

  public AlgaeIntakeIO_Sim() {

    // Motor Initialization
    pivotMotor = new TalonFX(AlgaeIntakeConstants.kPivotMotorCANID);
    pivotMotor.getConfigurator().apply(AlgaeIntakeConstants.kPivotMotorConfig);

    wheelsMotor = new TalonFX(AlgaeIntakeConstants.kWheelsMotorCANID);
    wheelsMotor.getConfigurator().apply(AlgaeIntakeConstants.kWheelsMotorConfig);
  }

  @Override
  public void updateInputs(AlgaeIntakeIOInputs inputs) {

    pivotMotor.setControl(pivotSetpoint);
    wheelsMotor.setControl(wheelsSetpoint);

    // Simulate pivot motor
    var pivotMotorSim = pivotMotor.getSimState();
    pivotMotorSim.setSupplyVoltage(12);
    pivotMotorModel.setInputVoltage(pivotMotorSim.getMotorVoltage());
    pivotMotorModel.update(0.02);
    pivotMotorSim.setRawRotorPosition(
        pivotMotorModel.getAngularPosition().times(AlgaeIntakeConstants.kPivotGearRatio));
    pivotMotorSim.setRotorVelocity(
        pivotMotorModel.getAngularVelocity().times(AlgaeIntakeConstants.kPivotGearRatio));

    // Simulate wheels motor
    var wheelsMotorSim = wheelsMotor.getSimState();
    wheelsMotorSim.setSupplyVoltage(12);
    wheelsMotorModel.setInputVoltage(wheelsMotorSim.getMotorVoltage());
    wheelsMotorModel.update(0.02);
    wheelsMotorSim.setRawRotorPosition(
        wheelsMotorModel.getAngularPosition().times(AlgaeIntakeConstants.kWheelsGearRatio));
    wheelsMotorSim.setRotorVelocity(
        wheelsMotorModel.getAngularVelocity().times(AlgaeIntakeConstants.kWheelsGearRatio));

    // Update pivot inputs
    inputs.pivotMotorVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.pivotMotorCurrent = pivotMotor.getSupplyCurrent().getValueAsDouble();
    inputs.pivotMotorPosition =
        pivotMotor.getPosition().getValueAsDouble() * 360; // Convert rotations to degrees
    inputs.pivotMotorVelocity =
        pivotMotor.getVelocity().getValueAsDouble() * 360; // Convert rps to dps
    inputs.pivotMotorAcceleration =
        pivotMotor.getAcceleration().getValueAsDouble() * 360; // Convert rps^2 to dps^2
    inputs.pivotPositionSetpoint = pivotSetpoint.Position * 360; // Convert rotations to degrees

    // Update wheels inputs
    inputs.wheelsMotorVoltage = wheelsMotor.getMotorVoltage().getValueAsDouble();
    inputs.wheelsMotorCurrent = wheelsMotor.getSupplyCurrent().getValueAsDouble();
    inputs.wheelsMotorTemp = wheelsMotor.getDeviceTemp().getValue().in(Fahrenheit);
  }

  @Override
  public void changePivotSetpoint(AlgaePivotSetpoint newSetpoint) {
    var degrees =
        MathUtil.clamp(
            newSetpoint.degrees,
            AlgaeIntakeConstants.kPivotMinAngle,
            AlgaeIntakeConstants.kPivotMaxAngle);
    pivotSetpoint.Position = degrees / 360.0; // Convert degrees to rotations
  }

  @Override
  public void changeWheelsSetpoint(AlgaeWheelsSetpoint newSetpoint) {
    wheelsSetpoint.Output = newSetpoint.volts;
  }
}
