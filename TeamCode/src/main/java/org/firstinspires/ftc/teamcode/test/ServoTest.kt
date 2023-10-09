package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name = "Servo Test", group = "Tests")
@Suppress("unused")
class ServoTest : LinearOpMode() {
    private lateinit var servo: Servo
    private var servoClass: Class<Servo> = Servo::class.java

    override fun runOpMode() {
        servo = hardwareMap.get(servoClass, "lclawservo")
        waitForStart()
        while (opModeIsActive()) {
            servo.position = servo.position + -gamepad1.left_stick_x * .005
        }
    }
}
