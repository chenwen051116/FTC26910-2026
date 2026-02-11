package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private final DcMotorEx turretMotor;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorEx.Direction.FORWARD);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getAngle() {
        // TODO
        return 0;
    }

    public void setAngle(double angleInRadians) {
        // TODO
    }

    public void center() {
        setAngle(0);
    }
}
