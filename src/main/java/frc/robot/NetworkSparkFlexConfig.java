package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.Preferences;

public class NetworkSparkFlexConfig {
    public static final class Config {
        public double ampLimit = 60.0;
        public double p = 0.0;
        public double i = 0.0;
        public double d = 0.0;
        public double ffK = 0.0;
    }

    String key;
    SparkFlex spark;
    Config config;

    public NetworkSparkFlexConfig(String key, SparkFlex spark, Config backup) {
        if (!Constants.TUNNING) {
            return;
        }

        this.key = key;
        this.spark = spark;
        this.config = backup;

        broadcast();
        update();
    }

    public void broadcast() {
        Preferences.setDouble(key + "ampLimit", config.ampLimit);
        Preferences.setDouble(key + "pid.p", config.p);
        Preferences.setDouble(key + "pid.i", config.i);
        Preferences.setDouble(key + "pid.d", config.d);
        Preferences.setDouble(key + "ff.k", config.ffK);
    }

    /**
     * Gathers the new config.
     * 
     * @return True if dirty, false otherwise.
     */
    private boolean gather() {
        boolean dirty = false;

        double p = Preferences.getDouble(key + "pid.p", config.p);
        dirty |= (p != config.p);
        config.p = p;

        double i = Preferences.getDouble(key + "pid.i", config.i);
        dirty |= (i != config.i);
        config.i = i;

        double d = Preferences.getDouble(key + "pid.d", config.d);
        dirty |= (d != config.d);
        config.d = d;

        double ampLimit = Preferences.getDouble(key + "ampLimit", config.ampLimit);
        dirty |= (ampLimit != config.ampLimit);
        config.ampLimit = ampLimit;

        double ffK = Preferences.getDouble(key + "ff.k", config.ffK);
        dirty |= (ffK != config.ffK);
        config.ffK = ffK;

        return dirty;
    }

    private void setConfig() {
        SparkBaseConfig newConfig = new SparkFlexConfig().smartCurrentLimit((int) config.ampLimit);

        newConfig.closedLoop
                .pid(config.p, config.i, config.d).feedForward.kV(config.ffK);

        spark.configure(
                newConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public void update() {
        if (!Constants.TUNNING) {
            return;
        }

        boolean dirty = gather();
        if (dirty) {
            setConfig();
        }
    }
}
