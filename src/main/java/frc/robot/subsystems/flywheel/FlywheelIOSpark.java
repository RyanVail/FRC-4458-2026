package frc.robot.subsystems.flywheel;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class FlywheelIOSpark implements FlywheelIO {
    SparkMax spark;

    public FlywheelIOSpark() {
        spark = new SparkMax(IntakeConstants.PORT, MotorType.kBrushless);
    }

    @Override
    public void setVoltage(double voltage) {
        spark.setVoltage(voltage);
    }

    @Override
    public double getVelocity() {
        return spark.getEncoder().getVelocity();
    }

    @Override
    public double getVoltage() {
        return spark.getBusVoltage();
    }
}
