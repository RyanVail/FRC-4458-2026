package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.HopperConstants;

public class HopperIOSpark implements HopperIO {
    SparkMax conveyorSpark;
    SparkFlex shooterSpark;

    public HopperIOSpark() {
        conveyorSpark = new SparkMax(HopperConstants.CONVEYOR_PORT, MotorType.kBrushless);
        shooterSpark = new SparkFlex(HopperConstants.SHOOTER_PORT, MotorType.kBrushless);
    }

    @Override
    public void setconveyorVoltage(double voltage) {
        conveyorSpark.setVoltage(voltage);
    }

    @Override
    public void setShooterVoltage(double voltage) {
        shooterSpark.setVoltage(voltage);
    }
}
