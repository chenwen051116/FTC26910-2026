package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class AutoDrivetrain extends SubsystemBase {
    //declare motors.. 声明，赋值...
    private final DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public static double power = 0.4;
    public static int fwd1 = -2000;
    public static int turn1 = 400;

    public static int fwd2 = 1000;
    public static int turn2 = 400;
    public static int turn3 = 400;
    public static int turn4 = 400;
    public static int fwd3 = 1000;
    public static int fwd4 = 1000;
    public static int fwd5 = 1000;
    //servos
    public int fltar = 0;
    public int frtar = 0;
    public int bltar = 0;
    public int brtar = 0;
    public AutoDrivetrain(HardwareMap hardwareMap) {      //Constructor,新建对象时需要
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void forward (int dis){
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        fltar+=dis;
        frtar+=dis;
        bltar+=dis;
        brtar+=dis;
        frontLeftMotor.setTargetPosition(fltar);
        frontRightMotor.setTargetPosition(frtar);
        backLeftMotor.setTargetPosition(bltar);
        backRightMotor.setTargetPosition(brtar);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void turn (int dis){
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        fltar-=dis;
        frtar+=dis;
        bltar-=dis;
        brtar+=dis;
        frontLeftMotor.setTargetPosition(fltar);
        frontRightMotor.setTargetPosition(frtar);
        backLeftMotor.setTargetPosition(bltar);
        backRightMotor.setTargetPosition(brtar);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    // 在 Drivetrain getter
    public double getFrontLeftPower() {
        return frontLeftMotor.getPower();
    }

    public double getFrontRightPower() {
        return frontRightMotor.getPower();
    }

    public double getBackLeftPower() {
        return backLeftMotor.getPower();
    }

    public double getBackRightPower() {
        return backRightMotor.getPower();
    }


}

