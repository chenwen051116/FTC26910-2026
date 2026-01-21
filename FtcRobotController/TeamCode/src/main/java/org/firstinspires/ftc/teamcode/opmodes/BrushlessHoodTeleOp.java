package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class BrushlessHoodTeleOp extends LinearOpMode {

    // 声明无刷舵机
    private Servo myBrushlessServo;

    @Override
    public void runOpMode() throws InterruptedException {

        // 初始化 ServoEx（无刷舵机）
        myBrushlessServo = hardwareMap.get(Servo.class, "my_brushless_servo");

        // 设置 ServoEx 控制的 PWM 最小值和最大值
        myBrushlessServo.setDirection(Servo.Direction.FORWARD); // 设定舵机方向（可以根据硬件调整）
        myBrushlessServo.setPosition(0.5); // 设置初始位置（这里假设舵机在中间位置）

        // 等待启动信号
        waitForStart();

        // 主控制循环
        while (opModeIsActive()) {

            // 模拟通过按钮控制舵机的不同角度
            if (gamepad1.a) {
                // 按下 A 键，舵机转到 0 度位置
                myBrushlessServo.setPosition(0.0);  // 相当于 PWM 占空比 0%
            } else if (gamepad1.b) {
                // 按下 B 键，舵机转到 90 度位置
                myBrushlessServo.setPosition(0.5);  // 相当于 PWM 占空比 50%
            } else if (gamepad1.x) {
                // 按下 X 键，舵机转到 180 度位置
                myBrushlessServo.setPosition(1.0);  // 相当于 PWM 占空比 100%
            } else {
                // 如果没有按下按钮，舵机保持当前角度
                myBrushlessServo.setPosition(myBrushlessServo.getPosition());
            }

            // 稍微延迟一下，避免过于频繁地更新舵机状态
            idle();
        }
    }
}
