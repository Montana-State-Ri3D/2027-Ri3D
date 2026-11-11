package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Queue;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class DriveModuleIOTalonFX implements DriveModuleIO {

  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final CANCoder encoder;
  private final TalonFXConfiguration id;

  private final Queue<Double> timestamps;

  private final double driveVoltage;
  private final Queue<Double> driveRelativePosMetersQueue;
  private final double driveVelMetersPerSecond;
  private final double driveDesiredVelMetersPerSecond;
  private final double driveCurrent;

  private final double turnVoltage;
  private final Queue<Double> turnRelativePosMetersQueue;
  private final StatusSignal<Angle> turnAbsolutePosMetersQueue;
  private final double turnVelRadiansPerSecond;
  private final double turnDesiredPosRotations;
  private final double turnCurrent;

  private LinearVelocity desiredVel = MetersPerSecond.of(0);

  //TODO: set up low level IO

  public DriveModuleIOTalonFX(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants,
    PhoenixOdometryThread thread
) {
    motor = new SparkMax(CanIDs.DRIVE_CAN_IDS[id], MotorType.kBrushless);
    encoder = motor.getEncoder();
  }

  @Override
  public void updateInputs(DriveModuleIOInputsAutoLogged inputs) {
    inputs.

    Logger.processInputs(DriveConstants.ROOT_TABLE + "/Modules/Module" + id, inputs);
  }

  public void setVoltage(Voltage volts) {
    motor.setVoltage(volts);
  }

  public void setVelocity(LinearVelocity vel) {
    desiredVel = vel;
    motor
        .getClosedLoopController()
        .setReference(
            Units.RadiansPerSecond.of(vel.div(DriveConstants.WHEEL_RAD).magnitude()).in(Units.RPM)
                / DriveConstants.GEAR_RATIO,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0);
  }

  public Distance getRelativePosition() {
    return Units.Meter.of(
        Units.Rotations.of(encoder.getPosition()).in(Units.Radians)
            * DriveConstants.WHEEL_RAD.in(Units.Meter));
  }

  public LinearVelocity getVelocity() {
    return Units.MetersPerSecond.of(
        Units.RPM.of(encoder.getVelocity()).in(Units.RadiansPerSecond)
            * DriveConstants.WHEEL_RAD.in(Units.Meter));
  }

  public void updateMotorConfig(double kV, double kP) {
    SparkMaxConfig config = DriveConstants.MOTOR_CONFIG(id);
    config.closedLoop.pidf(kP, 0, 0, kV);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
