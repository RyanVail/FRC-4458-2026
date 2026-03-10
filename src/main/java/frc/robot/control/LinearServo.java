package frc.robot.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;

public class LinearServo extends Servo {
    double speed;
    double length;

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
            2_000,
            1_800,
            1_500,
            1_200,
            1_000
        );
    }

    /**
     * Sets the target position of this servo in mm.
     */
    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, 0, length);
        setSpeed(setpoint / length * 2 - 1);
    }

    public double getPosition() {
        return (getSpeed() + 1) * 0.5 * length;
    }
}
