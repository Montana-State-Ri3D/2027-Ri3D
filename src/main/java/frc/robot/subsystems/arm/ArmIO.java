package frc.robot.subsystems.arm;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  class ArmInputs {
    public double angleDegrees;
    public double appliedOutput;
    public double current;
    public double tempCelcius;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setAngle(Rotation2d angle) {}

  public default void configMotor(double kP, double kD) {}

  public default void setSensorAngle(Rotation2d angle) {}

  public default boolean setIdleMode(IdleMode value) {
    return false;
  }
}
