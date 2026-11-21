package frc.robot.subsystems.example_elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants;
import frc.robot.Constants.ExampleElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {
    private final SparkFlex motor =
        new SparkFlex(Constants.CanIDs.ELEVATOR_LEAD_CAN_ID, MotorType.kBrushless);

    private final RelativeEncoder encoder = motor.getEncoder();

    public ElevatorIOReal() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.follow(motor, Constants.ElevatorConstants.FOLLOWER_INVERT);
    }
    
    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.heightInches = encoder.getPosition();
        inputs.velocityInchesPerSecond = encoder.getVelocity() * 60.0;
    }
    
    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

}