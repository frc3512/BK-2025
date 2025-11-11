package frc.robot.subsystems.algaeIntake;

import static edu.wpi.first.units.Units.Fahrenheit;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaePivotSetpoint;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaeWheelsSetpoint;

public class AlgaeIntakeIO_Real implements AlgaeIntakeIO {

  private TalonFX pivotMotor;
  private MotionMagicVoltage pivotSetpoint = new MotionMagicVoltage(0); // Mechanism Rotations

  private TalonFX wheelsMotor;
  private VoltageOut wheelsSetpoint = new VoltageOut(0); // Volts

  private Canandmag pivotEncoder;

  public AlgaeIntakeIO_Real() {

    // Motor Initialization
    pivotMotor = new TalonFX(AlgaeIntakeConstants.kPivotMotorCANID);
    pivotMotor.getConfigurator().apply(AlgaeIntakeConstants.kPivotMotorConfig);

    wheelsMotor = new TalonFX(AlgaeIntakeConstants.kWheelsMotorCANID);
    wheelsMotor.getConfigurator().apply(AlgaeIntakeConstants.kWheelsMotorConfig);

    // Absolute encoder Initialization
    pivotEncoder = new Canandmag(AlgaeIntakeConstants.kPivotEncoderCANID);

    // Optimize Pivot Motor Can Bus Utilization
    var pivotMotorVoltageSignal = pivotMotor.getMotorVoltage();
    var pivotMotorCurrentSignal = pivotMotor.getSupplyCurrent();
    var pivotMotorPositionSignal = pivotMotor.getPosition();
    var pivotMotorVelocitySignal = pivotMotor.getVelocity();
    var pivotMotorAccelerationSignal = pivotMotor.getAcceleration();

    pivotMotorVoltageSignal.setUpdateFrequency(50);
    pivotMotorCurrentSignal.setUpdateFrequency(50);
    pivotMotorPositionSignal.setUpdateFrequency(50);
    pivotMotorVelocitySignal.setUpdateFrequency(50);
    pivotMotorAccelerationSignal.setUpdateFrequency(50);

    // Optimize Wheels Motor Can Bus Utilization
    var wheelsMotorVoltageSignal = wheelsMotor.getMotorVoltage();
    var wheelsMotorCurrentSignal = wheelsMotor.getSupplyCurrent();
    var wheelsMotorTemperatureSignal = wheelsMotor.getDeviceTemp();

    wheelsMotorVoltageSignal.setUpdateFrequency(50);
    wheelsMotorCurrentSignal.setUpdateFrequency(50);
    wheelsMotorTemperatureSignal.setUpdateFrequency(50d / 4);

    pivotMotor.optimizeBusUtilization();
    wheelsMotor.optimizeBusUtilization();
    pivotMotor.setPosition(getAbsEncoderDeg() / 360);
  }

  public final double getAbsEncoderDeg() {
    return ((360.0 * pivotEncoder.getAbsPosition() + 180) % 360.0) - 180.0;
  }

  @Override
  public void updateInputs(AlgaeIntakeIOInputs inputs) {

    pivotMotor.setControl(pivotSetpoint);
    wheelsMotor.setControl(wheelsSetpoint);

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
