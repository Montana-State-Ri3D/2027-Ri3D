package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.DriveConstants;

public class DriveModule{
    private final DriveModuleIO io;
    private final int index; // 0 = FL, 1 = FR, 2 = BL, 3 = BR
    private final DriveModuleIOInputsAutoLogged inputs = new DriveModuleIOInputsAutoLogged();

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;

    public DriveModule(DriveModuleIO io, int index, ){
        this.io = io;
        this.index = index;

        driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + DriveConstants.MODULE_NAMES[index] + ".",
            AlertType.kError);
        turnDisconnectedAlert =
            new Alert(
                "Disconnected turn motor on module " + DriveConstants.MODULE_NAMES[index] + ".", AlertType.kError);
        turnEncoderDisconnectedAlert =
            new Alert(
                "Disconnected turn encoder on module " + DriveConstants.MODULE_NAMES[index] + ".",
                AlertType.kError);
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs(DriveConstants.ROOT_TABLE + "/Modules/" + DriveConstants.MODULE_NAMES[index], inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        SwerveModulePosition[] odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * DriveConstants.WHEEL_RAD.in(Units.Meter);
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
        turnEncoderDisconnectedAlert.set(!inputs.encoderConnected);
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        state.optimize(inputs.turnAbsolutePosition);
        state.cosineScale(inputs.turnAbsolutePosition);

        // Apply setpoints
        io.setDriveVelocity(Units.MetersPerSecond.of(state.speedMetersPerSecond));
        io.setTurnPosition(state.angle);
    }

    public void stop(){
        io.setDriveVoltage(Units.Volts.of(0));
        io.setTurnVoltage(Units.Volts.of(0));
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(inputs.driveVelMetersPerSecond, Rotation2d.fromRotations(inputs.turnRelativePosRotations))
    }
}
