package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

public abstract class Overridable extends SubsystemBase {
    private boolean isOverrideDriver = false;

    public void overrideDriver() {
        isOverrideDriver = true;
        runWhenStartingOverride();
    }

    public void stopOverrideDriver() {
        isOverrideDriver = false;
        runWhenStoppingOverride();
    }

    public void toggleOverrideDriver() {
        if (isOverrideDriver) {
            stopOverrideDriver();
        } else {
            overrideDriver();
        }
    }

    public boolean isOverriding() {
        return isOverrideDriver;
    }
    public void runWithoutOverride() {}
    public void alwaysRunning() {}
    public void runWhenStartingOverride() {}
    public void runWhenStoppingOverride() {}
    public void periodic() {
        if (!isOverrideDriver) {
            runWithoutOverride();
        }
        alwaysRunning();
    }
}
