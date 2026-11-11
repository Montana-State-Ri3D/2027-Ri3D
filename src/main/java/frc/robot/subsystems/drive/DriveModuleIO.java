package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface DriveModuleIO {
  @AutoLog
  public static class DriveModuleIOInputs {
    public boolean driveConnected = false;
    public double driveVoltage;
    public double driveRelativePosMeters;
    public double driveVelMetersPerSecond;
    public double driveDesiredVelMetersPerSecond;
    public double driveCurrent;

    public boolean turnConnected = false;
    public double turnVoltage;
    public double turnRelativePosRotations;
    public double turnVelRadiansPerSecond;
    public double turnDesiredPosRotations;
    public double turnCurrent;

    public boolean encoderConnected = false;
    public Rotation2d turnAbsolutePosition = Rotation2d.kZero;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  public default void updateInputs(DriveModuleIOInputs inputs) {}

  public default void setDriveVoltage(Voltage volts) {}

  public default void setDriveVelocity(LinearVelocity vel) {}

  public default void setTurnVoltage(Voltage volts) {}

  public default void setTurnPosition(Rotation2d angle) {}

  public default void updateMotorConfig(double kV, double kP) {}
}
