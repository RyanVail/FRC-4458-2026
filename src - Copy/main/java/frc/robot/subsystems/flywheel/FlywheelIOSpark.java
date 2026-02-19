package frc.robot.subsystems.flywheel;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.FlyWheelConstants;

public class FlywheelIOSpark implements FlywheelIO {
    SparkMax leftSpark;
    SparkMax rightSpark;

    public FlywheelIOSpark() {
        leftSpark = new SparkMax(FlyWheelConstants.LEFT_PORT, MotorType.kBrushless);
        rightSpark = new SparkMax(FlyWheelConstants.RIGHT_PORT, MotorType.kBrushless);
    }

    @Override
    public void setLeftVoltage(double voltage) {
        leftSpark.setVoltage(voltage);
    }

    @Override
    public void setRightVoltage(double voltage) {
        rightSpark.setVoltage(voltage);
    }

    @Override
    public double getLeftVelocity() {
        return leftSpark.getEncoder().getVelocity();
    }

    @Override
    public double getRightVelocity() {
        return rightSpark.getEncoder().getVelocity();
    }

    @Override
    public double getLeftVoltage() {
        return leftSpark.getBusVoltage();
    }

    @Override
    public double getRightVoltage() {
        return rightSpark.getBusVoltage();
    }
}
