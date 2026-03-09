package frc.robot.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;

public class LinearServo extends Servo {
    double speed;
    double length;

    double targetPos;
    double pos;

    /**
     * Creates a class able to control a linear servo.
     * 
     * @param channel The PWM channel
     * @param length  The max length of the servo in mm.
     * @param speed   The speed of the servo in mm / sec.
     */
    public LinearServo(int channel, double length, double speed) {
        super(channel);
        this.length = length;
        this.speed = speed;
        setBoundsMicroseconds(
            2_000_000,
            1_800_000,
            1_500_000,
            1_200_000,
            1_000_000
        );
    }

    /**
     * Sets the target position of this servo in mm.
     */
    public void setSetpoint(double setpoint) {
        targetPos = MathUtil.clamp(setpoint, 0, length);
        setPosition(targetPos / length);
    }

    /**
     * Checks if the servo is its setpoint within some bound.
     */
    public boolean isFinished(double bounds) {
        return Math.abs(pos - targetPos) <= bounds;
    }
}
