package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;
public interface FlywheelIO {
    @AutoLog
    class FlywheelInputs{ 
        public double flyWheelVelocity;
    }
    public default void updateInputs(FlywheelInputs inputs){}
    public default void setVoltage(double volts){}
}
