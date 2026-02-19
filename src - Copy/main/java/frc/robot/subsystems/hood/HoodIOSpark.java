package frc.robot.subsystems.hood;

import frc.robot.Constants.HoodConstants;
import frc.robot.control.LinearServo;

public class HoodIOSpark implements HoodIO {
    LinearServo leftServo;
    LinearServo rightServo;

    public HoodIOSpark() {
        leftServo = new LinearServo(
            HoodConstants.LEFT_CHANNEL,
            HoodConstants.LENGTH,
            HoodConstants.SPEED
        );

        rightServo = new LinearServo(
            HoodConstants.RIGHT_CHANNEL,
            HoodConstants.LENGTH,
            HoodConstants.SPEED
        );
    }

    public void periodic() {
        leftServo.update();
        rightServo.update();
    }

    public double getLeftPosition() {
        return leftServo.getPosition();
    }

    public double getRightPosition() {
        return rightServo.getPosition();
    }

    public void setSetpoint(double setpoint) {
        leftServo.setSetpoint(setpoint);
        rightServo.setSetpoint(setpoint);
    }
}
