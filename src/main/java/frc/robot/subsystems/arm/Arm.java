// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ControlMode;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  // Logging

  private static final LoggerGroup logGroup = LoggerGroup.build(ArmConstants.ROOT_TABLE);

  private static final LoggerEntry.Decimal logTargetAngle =
      logGroup.buildDecimal("extenderTargetPosInches");
  private static final LoggerEntry.EnumValue<ControlMode> logControlMode =
      logGroup.buildEnum("ControlMode");

  // Tunable numbers

  private static final TunableNumberGroup group =
      new TunableNumberGroup(ArmConstants.ROOT_TABLE);

  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kD = group.build("kD");

  private final LoggedTunableNumber toleranceDegrees = group.build("toleranceDegrees", 0.1);

  private Rotation2d armTargetPos = Rotation2d.kZero;

  // Motion constants
  // TODO: tune constants
  static {
    if (Constants.currentMode == Mode.SIM) {
      kP.initDefault(0.1);
      kD.initDefault(0);
    } else if (Constants.currentMode == Mode.REAL) {
      kP.initDefault(0.5);
      kD.initDefault(0);
    }
  }

  private final ArmIO io;
  private final ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();

  private AngularVelocity targetVel = Units.RPM.zero();

  /** Creates a new ArmSubsystem. */
  public Arm(ArmIO io) {
    this.io = io;

    config();

    io.setVoltage(0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(ArmConstants.ROOT_TABLE, inputs);

    // Updating tunable numbers
    var hc = hashCode();
    if (kP.hasChanged(hc) || kD.hasChanged(hc)) config();
  }

  // Setters

  public void setExtenderPercentOut(double percent) {
    io.setVoltage(percent * Constants.MAX_VOLTAGE.in(Units.Volts));
  }

  public void setAngle(Rotation2d angle) {
    if (angle.getDegrees() < ArmConstants.MIN_ANGLE.getDegrees() || angle.getDegrees() > ArmConstants.MAX_ANGLE.getDegrees()) {
      System.out.println("\u001B[33mWARNING: Arm Angle Clamped\u001B[0m");
      angle =
          Rotation2d.fromDegrees(
              MathUtil.clamp(
                  angle.getDegrees(), ArmConstants.MIN_ANGLE.getDegrees(), ArmConstants.MAX_ANGLE.getDegrees()));
    }
    io.setAngle(angle);
    armTargetPos = angle;
    logTargetAngle.info(armTargetPos.getDegrees());
  }

  public boolean setIdleMode(IdleMode value) {
    return io.setIdleMode(value);
  }

  private void config() {
    io.configMotor(kP.get(), kD.get());
  }

  public void resetSensorToHomePosition() {
    io.setSensorAngle(ArmConstants.HOME_ANGLE);
  }

  // Getters

  public boolean isAtTarget() {
    return isAtTarget(armTargetPos);
  }

  public boolean isAtTarget(Rotation2d angle) {
    return Math.abs(angle.getDegrees() - inputs.angleDegrees)
        <= toleranceDegrees.get();
  }
}
