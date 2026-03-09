package frc.robot.subsystems.flywheel;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.Constants.FlyWheelConstants;
import frc.robot.NetworkSparkFlexConfig.Config;
import frc.robot.NetworkSparkFlexConfig;

public class FlywheelIOSim implements FlywheelIO {
    SparkFlex leftSpark;
    SparkFlexSim leftSparkSim;
    FlywheelSim leftMechSim;

    SparkFlex rightSpark;
    SparkFlexSim rightSparkSim;
    FlywheelSim rightMechSim;

    NetworkSparkFlexConfig leftNetworkConfig;
    NetworkSparkFlexConfig rightNetworkConfig;

    public FlywheelIOSim() {
        DCMotor leftGearbox = DCMotor.getNEO(1);
        leftSpark = new SparkFlex(FlyWheelConstants.LEFT_PORT, MotorType.kBrushless);
        leftSparkSim = new SparkFlexSim(leftSpark, leftGearbox);
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
        rightSpark = new SparkFlex(FlyWheelConstants.RIGHT_PORT, MotorType.kBrushless);
        rightSparkSim = new SparkFlexSim(rightSpark, rightGearbox);
        rightMechSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                rightGearbox,
                FlyWheelConstants.MOI,
                FlyWheelConstants.GEARING
            ),
            rightGearbox,
            FlyWheelConstants.STD_DEVS
        );

        Config config = new Config();
        leftNetworkConfig = new NetworkSparkFlexConfig(
            Flywheel.LPREFIX + "left/",
            leftSpark,
            config
        );

        rightNetworkConfig = new NetworkSparkFlexConfig(
            Flywheel.LPREFIX + "right/",
            rightSpark,
            config
        );
    }

    @Override
    public void setLeftVelocitySetpoint(double velocity) {
        leftSpark.getClosedLoopController().setSetpoint(
            velocity,
            ControlType.kVelocity
        );
    }

    @Override
    public void setRightVelocitySetpoint(double velocity) {
        rightSpark.getClosedLoopController().setSetpoint(
            velocity,
            ControlType.kVelocity
        );
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
    public double getLeftPosition() {
        return leftSpark.getEncoder().getPosition();
    }

    @Override
    public double getRightPosition() {
        return rightSpark.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        leftNetworkConfig.update();
        rightNetworkConfig.update();
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
