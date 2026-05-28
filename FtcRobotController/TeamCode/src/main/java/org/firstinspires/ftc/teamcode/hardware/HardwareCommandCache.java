package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

public final class HardwareCommandCache {
    public static double motorPowerTolerance = 0.0005;
    public static double servoPositionTolerance = 0.0005;

    private static final Map<DcMotorSimple, Double> motorPowerCache = new IdentityHashMap<>();
    private static final Map<Servo, Double> servoPositionCache = new IdentityHashMap<>();

    private HardwareCommandCache() {
    }

    public static void resetCommandCache() {
        motorPowerCache.clear();
        servoPositionCache.clear();
    }

    public static List<LynxModule> enableManualBulkCaching(HardwareMap hardwareMap) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hub.clearBulkCache();
        }
        return allHubs;
    }

    public static List<LynxModule> enableAutoBulkCaching(HardwareMap hardwareMap) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        return allHubs;
    }

    public static void clearBulkCache(List<LynxModule> allHubs) {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public static void setMotorPower(DcMotorSimple motor, double power) {
        Double previousPower = motorPowerCache.get(motor);
        if (previousPower == null || Math.abs(previousPower - power) > motorPowerTolerance) {
            motor.setPower(power);
            motorPowerCache.put(motor, power);
        }
    }

    public static void setServoPosition(Servo servo, double position) {
        Double previousPosition = servoPositionCache.get(servo);
        if (previousPosition == null || Math.abs(previousPosition - position) > servoPositionTolerance) {
            servo.setPosition(position);
            servoPositionCache.put(servo, position);
        }
    }

    public static void forceSetMotorPower(DcMotorSimple motor, double power) {
        motor.setPower(power);
        motorPowerCache.put(motor, power);
    }

    public static void forceSetServoPosition(Servo servo, double position) {
        servo.setPosition(position);
        servoPositionCache.put(servo, position);
    }
}
