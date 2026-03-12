package frc.robot.subsystems.intake;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
    SparkFlex spark;
    SparkFlexSim sparkSim;

    SparkMax rotSpark;
    SparkMaxSim rotSparkSim;

    public IntakeIOSim() {
        DCMotor gearbox = DCMotor.getNeoVortex(1);
        spark = new SparkFlex(IntakeConstants.PORT, MotorType.kBrushless);
        sparkSim = new SparkFlexSim(spark, gearbox);

        DCMotor rotGearbox = DCMotor.getNEO(1);
        rotSpark = new SparkMax(IntakeConstants.ROT_PORT, MotorType.kBrushless);
        rotSparkSim = new SparkMaxSim(rotSpark, rotGearbox);
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
