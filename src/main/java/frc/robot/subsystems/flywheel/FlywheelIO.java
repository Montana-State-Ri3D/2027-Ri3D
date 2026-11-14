public interface FlywheelIO {
    @AutoLog
    class FlywheelImputs{ 
        public double flyWheelVelocity;
    }
    public default void updateInputs(FlywheelInputs inputs){}
    public default void setVoltage(double volts){}
}
