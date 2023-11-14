package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name = "Servo Test", group = "Tests")
@Suppress("unused")
class ServoTest : LinearOpMode() {
    private lateinit var lservo: Servo
    private lateinit var rservo: Servo
    private val lastGamepad = Gamepad()
    private val gamepad = Gamepad()

    override fun runOpMode() {
        this.telemetry.msTransmissionInterval = 1
        lservo = hardwareMap.get(Servo::class.java, "lclawservo")
        rservo = hardwareMap.get(Servo::class.java, "rclawservo")
        lservo.position = 0.5
        rservo.position = 0.5
        this.waitForStart()
        while (this.opModeIsActive()) {
            lastGamepad.copy(gamepad)
            gamepad.copy(gamepad1)
            // lservo.position = (sin(time) + 1) / 2
//            lservo.position += -gamepad1.left_stick_x * 0.005
//            rservo.position += -gamepad1.right_stick_x * 0.005
            if (gamepad.a && !lastGamepad.a) lservo.position = 0.8
            else if (gamepad.b && !lastGamepad.b) lservo.position = 0.5
            if (gamepad.x && !lastGamepad.x) rservo.position = 0.8
            else if (gamepad.y && !lastGamepad.y) rservo.position = 0.5
            this.telemetry.addLine("lpos: " + lservo.position)
            this.telemetry.addLine("rpos: " + rservo.position)
            this.telemetry.update()
        }
    }
}
