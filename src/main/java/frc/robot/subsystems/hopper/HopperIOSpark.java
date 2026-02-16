package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.HopperConstants;

public class HopperIOSpark implements HopperIO {
    SparkMax leftSpark;
    SparkMax rightSpark;

    public HopperIOSpark() {
        leftSpark = new SparkMax(HopperConstants.LEFT_PORT, MotorType.kBrushless);
        rightSpark = new SparkMax(HopperConstants.RIGHT_PORT, MotorType.kBrushless);
    }

    @Override
    public void setVoltage(double voltage) {
        leftSpark.setVoltage(voltage);
        rightSpark.setVoltage(voltage);
    }
}
