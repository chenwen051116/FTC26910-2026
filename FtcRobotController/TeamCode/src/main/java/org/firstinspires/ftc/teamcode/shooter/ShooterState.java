package org.firstinspires.ftc.teamcode.shooter;

public class ShooterState {
    public final double turretAngle;
    public final double hoodPosition;
    public final double flywheelRPM;

    public ShooterState(double turretAngle, double hoodPosition, double flywheelRPM ) {
        this.turretAngle = turretAngle;
        this.hoodPosition = hoodPosition;
        this.flywheelRPM = flywheelRPM;
    }
}
