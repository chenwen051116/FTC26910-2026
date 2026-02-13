package org.firstinspires.ftc.teamcode.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter extends SubsystemBase {
    private final boolean isBlue;

    private final Flywheel flywheel;
    private final Turret turret;
    private final Hood hood;
    private ShooterState shooterState;


    // Constructor
    public Shooter(boolean isBlue, HardwareMap hardwareMap ) {
        this.isBlue = isBlue;

        // Initialize flywheel class
        DcMotorEx flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheel_1");
        DcMotorEx flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheel_2");
        flywheel = new Flywheel(flywheelMotor1, flywheelMotor2);

        // Initialize turret class
        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turret = new Turret(turretMotor);

        // Initialize hood
        Servo hoodServo = hardwareMap.get(Servo.class, "hood");
        hood = new Hood(hoodServo);
    }

    // Getters for debugging
    // Get the current angle of turret in radians
    public double getTurretAngle() {
        return turret.getAngle();
    }

    // Get the current position of hood from 0 to 1
    public double getHoodPosition() {
        return hood.getPosition();
    }

    // Get the current RPM of flywheel
    public double getFlywheelRPM() {
        return flywheel.getRPM();
    }

    // Set the shooter state and set the power of different components
    public void setShooterState(ShooterState targetShooterState ) {
        shooterState = targetShooterState;
    }

    // Update in every single tick of loop
    @Override
    public void periodic() {
        turret.setAngle(shooterState.turretAngle );
        flywheel.setRPM(shooterState.flywheelRPM );
        hood.setPosition(shooterState.hoodPosition );
    }

}
