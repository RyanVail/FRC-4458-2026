package frc.robot.subsystems.flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.FlyWheelConstants;

public class FlywheelIOSpark implements FlywheelIO {
    SparkFlex leftSpark;
    SparkFlex rightSpark;

    public FlywheelIOSpark() {
        leftSpark = new SparkFlex(FlyWheelConstants.LEFT_PORT, MotorType.kBrushless);
        rightSpark = new SparkFlex(FlyWheelConstants.RIGHT_PORT, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.closedLoop.pid(FlyWheelConstants.P, 0.0, 0.0);
        config.closedLoop.feedForward.kV(FlyWheelConstants.FF_V);

        leftSpark.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        rightSpark.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
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
}
