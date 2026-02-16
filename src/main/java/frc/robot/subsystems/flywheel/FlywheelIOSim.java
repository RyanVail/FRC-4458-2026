package frc.robot.subsystems.flywheel;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.Constants.FlyWheelConstants;

public class FlywheelIOSim implements FlywheelIO {
    SparkMax leftSpark;
    SparkMaxSim leftSparkSim;
    FlywheelSim leftMechSim;

    SparkMax rightSpark;
    SparkMaxSim rightSparkSim;
    FlywheelSim rightMechSim;

    public FlywheelIOSim() {
        DCMotor leftGearbox = DCMotor.getNEO(1);
        leftSpark = new SparkMax(FlyWheelConstants.LEFT_PORT, MotorType.kBrushless);
        leftSparkSim = new SparkMaxSim(leftSpark, leftGearbox);
        leftMechSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                leftGearbox,
                FlyWheelConstants.MOI,
                FlyWheelConstants.GEARING
            ),
            leftGearbox,
            FlyWheelConstants.STD_DEVS
        );

        DCMotor rightGearbox = DCMotor.getNEO(1);
        rightSpark = new SparkMax(FlyWheelConstants.RIGHT_PORT, MotorType.kBrushless);
        rightSparkSim = new SparkMaxSim(rightSpark, rightGearbox);
        rightMechSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                rightGearbox,
                FlyWheelConstants.MOI,
                FlyWheelConstants.GEARING
            ),
            rightGearbox,
            FlyWheelConstants.STD_DEVS
        );
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
        return leftSpark.getAppliedOutput() * RoboRioSim.getVInVoltage();
    }

    @Override
    public double getRightVoltage() {
        return rightSpark.getAppliedOutput() * RoboRioSim.getVInVoltage();
    }

    @Override
    public void simulationPeriodic() {
        leftMechSim.setInput(leftSparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        leftMechSim.update(Constants.LOOP_TIME);

        leftSparkSim.iterate(
            leftMechSim.getAngularVelocityRPM(),
            RoboRioSim.getVInVoltage(),
            Constants.LOOP_TIME
        );

        rightMechSim.setInput(rightSparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        rightMechSim.update(Constants.LOOP_TIME);

        rightSparkSim.iterate(
            rightMechSim.getAngularVelocityRPM(),
            RoboRioSim.getVInVoltage(),
            Constants.LOOP_TIME
        );

        // TODO: This should be done somewhere else.
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                leftMechSim.getCurrentDrawAmps(),
                rightMechSim.getCurrentDrawAmps()
            )
        );
    }
}
