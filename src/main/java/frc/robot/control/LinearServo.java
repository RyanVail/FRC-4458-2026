package frc.robot.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

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
                (int) Units.secondsToMilliseconds(2.0),
                (int) Units.secondsToMilliseconds(1.8),
                (int) Units.secondsToMilliseconds(1.5),
                (int) Units.secondsToMilliseconds(1.2),
                (int) Units.secondsToMilliseconds(1.0));
    }

    /**
     * Sets the target position of this servo in mm.
     */
    public void setSetpoint(double setpoint) {
        targetPos = MathUtil.clamp(setpoint, 0, length);
        setSpeed((targetPos / length * 2) - 1);
    }

    double lastTime = 0;

    public void update() {
        double dt = Timer.getFPGATimestamp() - lastTime;
        if (pos > targetPos + speed * dt) {
            pos -= speed * dt;
        } else if (pos < targetPos - speed * dt) {
            pos += speed * dt;
        } else {
            pos = targetPos;
        }
    }

    /**
     * Gets the position of this servo in mm.
     */
    public double getPosition() {
        return pos;
    }

    /**
     * Checks if the servo is its setpoint within some bound.
     */
    public boolean isFinished(double bounds) {
        return Math.abs(pos - targetPos) <= bounds;
    }
}
