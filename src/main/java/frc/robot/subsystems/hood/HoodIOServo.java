package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.HoodConstants;
import frc.robot.control.LinearServo;

public class HoodIOServo implements HoodIO {
    LinearServo leftServo;
    LinearServo rightServo;

    public HoodIOServo() {
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
        Logger.recordOutput(Hood.LPREFIX + "leftPos", leftServo.getPosition());
        Logger.recordOutput(Hood.LPREFIX + "rightPos", rightServo.getPosition());
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
