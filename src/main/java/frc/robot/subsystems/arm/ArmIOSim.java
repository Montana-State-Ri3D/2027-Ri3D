package frc.robot.subsystems.arm;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import frc.lib.teamBSR.GenericMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmIOSim implements ArmIO {

  private final GenericMotorSim motor =
      new GenericMotorSim(
          DCMotor.getNEO(1),
          ArmConstants.GEAR_RATIO,
          ArmConstants.MOI);

  @Override
  public void updateInputs(ArmInputs inputs) {
    motor.update(Constants.defaultPeriod);
    inputs.appliedOutput = motor.getVoltage();
    inputs.angleDegrees =
        motor.getPosition().in(Units.Degrees) / ArmConstants.GEAR_RATIO;
    inputs.current = motor.getCurrent().in(Units.Amp);
  }

  @Override
  public void setAngle(Rotation2d angle) {
    motor.setPosition(
        Units.Rotation.of(angle.getRotations() * ArmConstants.GEAR_RATIO));
  }

  @Override
  public void configMotor(double kP, double kD) {
    motor.setConfig(kP, 0, kD, 0, 0);
  }

  @Override
  public void setSensorAngle(Rotation2d angle) {
    motor.setState(
        Units.Rotations.of(angle.getRotations() * ArmConstants.GEAR_RATIO)
            .in(Units.Radian),
        0);
  }

  @Override
  public boolean setIdleMode(IdleMode value) {
    return true;
  }
}
