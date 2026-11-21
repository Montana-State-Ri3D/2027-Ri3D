package frc.robot.subsystems.example_elevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;
public interface ElevatorIO {

    /** Contains all of the input data received from hardware. */
    @AutoLog
    class ElevatorInputs {
        public double heightInches;
        public double velocityInchesPerSecond;
        public double appliedOutput;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorInputs inputs) {}

    public default void setVoltage(double volts) {}
    
}