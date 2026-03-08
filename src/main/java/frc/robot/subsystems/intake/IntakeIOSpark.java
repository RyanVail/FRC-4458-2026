package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class IntakeIOSpark implements IntakeIO {
    SparkFlex spark;
    SparkMax rotSpark;

    public IntakeIOSpark() {
        spark = new SparkFlex(IntakeConstants.PORT, MotorType.kBrushless);
        rotSpark = new SparkMax(IntakeConstants.ROT_PORT, MotorType.kBrushless);
    }

    @Override
    public void setVoltage(double voltage) {
        spark.setVoltage(voltage);
    }

    @Override
    public void setRotVoltage(double voltage) {
        rotSpark.setVoltage(voltage);
    }

    public double getRotPosition() {
        return rotSpark.getEncoder().getPosition();
    }

    public double getRotVelocity() {
        return rotSpark.getEncoder().getVelocity();
    }

    @Override
    public double getVelocity() {
        return spark.getEncoder().getVelocity();
    }

    @Override
    public double getVoltage() {
        return spark.getBusVoltage();
    }

    @Override
    public double getPosition() {
        return spark.getEncoder().getPosition();
    }
}
