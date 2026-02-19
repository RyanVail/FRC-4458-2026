package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Preferences;

public final class PIDSupplier implements Supplier<PIDController> {
    String key;
    PIDController value;

    public PIDSupplier(String key, PIDConstants backup) {
        if (Constants.TUNNING) {
            this.key = key;

            // When constants get updated this will load in the correct value.
            Constants.registerConstant(key, backup);

            if (!Preferences.containsKey(key + ".p")) {
                loadConstants(key, backup);
            }
        }

        PIDConstants constants = gatherConstants(key);
        this.value = new PIDController(constants.kP, constants.kI, constants.kD);
    }

    public static final PIDConstants gatherConstants(String key) {
        double p = Preferences.getDouble(key + ".p", 0.0);
        double i = Preferences.getDouble(key + ".i", 0.0);
        double d = Preferences.getDouble(key + ".d", 0.0);
        return new PIDConstants(p, i, d);
    }

    public static final void loadConstants(String key, PIDConstants constants)
    {
        Preferences.setDouble(key + ".p", constants.kP);
        Preferences.setDouble(key + ".i", constants.kI);
        Preferences.setDouble(key + ".d", constants.kD);
    }

    public final PIDController get() {
        if (Constants.TUNNING) {
            PIDConstants c = gatherConstants(key);
            if (this.value.getP() != c.kP || this.value.getI() != c.kI || this.value.getD() != c.kD) {
                this.value.setPID(c.kP, c.kI, c.kD);
            }
        }

        return this.value;
    }
}