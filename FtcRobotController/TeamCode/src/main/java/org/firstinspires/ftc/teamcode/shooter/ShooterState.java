package org.firstinspires.ftc.teamcode.shooter;

public class ShooterState {
    public final double turretAngle;
    public final double hoodAngle;
    public final double rpm;

    public ShooterState(double turretAngle, double hoodAngle, double rpm) {
        this.turretAngle = turretAngle;
        this.hoodAngle = hoodAngle;
        this.rpm = rpm;
    }
}
