
package frc.robot.subsystems.example_elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    /** Creates a new ElevatorSubsystem. */
    public Elevator(ElevatorIO io) {
        this.io = io;

        io.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(ElevatorConstants.ROOT_TABLE, inputs);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }
}