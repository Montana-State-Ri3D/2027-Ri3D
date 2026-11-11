package frc.robot.subsystems.arm;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmIOReal implements ArmIO {

  private final SparkMax motor =
      new SparkMax(Constants.CanIDs.ARM_CAN_ID, MotorType.kBrushless);

  private SparkMaxConfig config = ArmConstants.MOTOR_CONFIG();

  private final RelativeEncoder encoder = motor.getEncoder();

  public ArmIOReal() {}

  @Override
  public void updateInputs(ArmInputs inputs) {
    inputs.angleDegrees = encoder.getPosition();
    inputs.appliedOutput = motor.getAppliedOutput();
    inputs.current = motor.getOutputCurrent();
    inputs.tempCelcius = motor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setAngle(Rotation2d angle) {
    motor.getClosedLoopController().setReference(angle.getDegrees(), ControlType.kPosition);
  }

  @Override
  public void configMotor(double kP, double kD) {
    config.closedLoop.pid(kP, 0, kD);
    motor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setSensorAngle(Rotation2d angle) {
    encoder.setPosition(angle.getDegrees());
  }

  @Override
  public boolean setIdleMode(IdleMode value) {
    config.idleMode(IdleMode.kCoast);
    return motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
            == REVLibError.kOk;
  }
}
