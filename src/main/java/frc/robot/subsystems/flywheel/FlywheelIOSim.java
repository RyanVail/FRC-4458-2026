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
    SparkMax spark;
    SparkMaxSim sparkSim;
    FlywheelSim mechSim;

    public FlywheelIOSim() {
        DCMotor gearbox = DCMotor.getNEO(1);
        spark = new SparkMax(FlyWheelConstants.PORT, MotorType.kBrushless);
        sparkSim = new SparkMaxSim(spark, gearbox);
        mechSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                gearbox,
                FlyWheelConstants.MOI,
                FlyWheelConstants.GEARING
            ),
            gearbox,
            FlyWheelConstants.STD_DEVS
        );
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
        return spark.getAppliedOutput() * RoboRioSim.getVInVoltage();
    }

    @Override
    public void simulationPeriodic() {
        mechSim.setInput(sparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        mechSim.update(Constants.LOOP_TIME);

        sparkSim.iterate(
            mechSim.getAngularVelocityRPM(),
            RoboRioSim.getVInVoltage(),
            Constants.LOOP_TIME
        );

        // TODO: This should be done somewhere else.
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                mechSim.getCurrentDrawAmps()
            )
        );
    }
}
