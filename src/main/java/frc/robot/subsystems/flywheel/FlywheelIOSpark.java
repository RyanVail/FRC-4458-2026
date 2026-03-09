package frc.robot.subsystems.flywheel;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.NetworkSparkFlexConfig;
import frc.robot.Constants.FlyWheelConstants;
import frc.robot.NetworkSparkFlexConfig.Config;

public class FlywheelIOSpark implements FlywheelIO {
    SparkFlex leftSpark;
    SparkFlex rightSpark;

    NetworkSparkFlexConfig leftNetworkConfig;
    NetworkSparkFlexConfig rightNetworkConfig;

    public FlywheelIOSpark() {
        leftSpark = new SparkFlex(FlyWheelConstants.LEFT_PORT, MotorType.kBrushless);
        rightSpark = new SparkFlex(FlyWheelConstants.RIGHT_PORT, MotorType.kBrushless);

        Config config = new Config();
        config.p = 0.005;
        config.ffK = 0.02;

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
    public void periodic() {
        leftNetworkConfig.update();
        rightNetworkConfig.update();
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
}
