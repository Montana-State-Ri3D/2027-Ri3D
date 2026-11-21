package frc.robot.subsystems.flywheel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO.IntakeInputs;
public class FlywheelIOReal implements FlywheelIO{
   private final SparkMax motor =
      new SparkMax(Constants.CanIDs.INTAKE_EXTENDER_CAN_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    @Overide
    public void updateInputs(FlywheelInputs inputs) {
        inputs.flyWheelVelocity = encoder.getVelocity();
    }
    @Overide    
    public void setVoltage(double volts) {
            motor.setVoltage(volts);
    }
   
    }
