package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.HopperConstants;

public class HopperIOSpark implements HopperIO {
    SparkMax conveyorSpark;
    SparkMax shooterSpark;

    public HopperIOSpark() {
        conveyorSpark = new SparkMax(HopperConstants.CONVEYOR_PORT, MotorType.kBrushless);
        shooterSpark = new SparkMax(HopperConstants.SHOOTER_PORT, MotorType.kBrushless);
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
