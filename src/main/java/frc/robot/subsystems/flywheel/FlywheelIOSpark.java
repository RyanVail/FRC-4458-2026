package frc.robot.subsystems.flywheel;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.FlyWheelConstants;

public class FlywheelIOSpark implements FlywheelIO {
    SparkFlex leftSpark;
    SparkFlex rightSpark;
    SparkMax middleSpark;

    public FlywheelIOSpark() {
        leftSpark = new SparkFlex(FlyWheelConstants.LEFT_PORT, MotorType.kBrushless);
        rightSpark = new SparkFlex(FlyWheelConstants.RIGHT_PORT, MotorType.kBrushless);
        middleSpark = new SparkMax(41, MotorType.kBrushless);
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
    public void setMiddleVoltage(double voltage) {
        middleSpark.setVoltage(voltage);
    }

    // 1.5 is our new gear ratio
    @Override
    public double getLeftVelocity() {
        return leftSpark.getEncoder().getVelocity() / 1.5;
    }

    @Override
    public double getRightVelocity() {
        return rightSpark.getEncoder().getVelocity() / 1.5;
    }

    @Override
    public double getMiddleVelocity() {
        return middleSpark.getEncoder().getVelocity() / 1.5;
    }

    @Override
    public double getLeftPosition() {
        return leftSpark.getEncoder().getPosition() / 1.5;
    }

    @Override
    public double getRightPosition() {
        return rightSpark.getEncoder().getPosition() / 1.5;
    }

    @Override
    public double getMiddlePosition() {
        return middleSpark.getEncoder().getPosition() / 1.5;
    }

    @Override
    public double getLeftVoltage() {
        return leftSpark.getBusVoltage() * leftSpark.get();
    }

    @Override
    public double getRightVoltage() {
        return rightSpark.getBusVoltage() * rightSpark.get();
    }

    @Override
    public double getMiddleVoltage() {
        return middleSpark.getBusVoltage() * middleSpark.get();
    }
}
