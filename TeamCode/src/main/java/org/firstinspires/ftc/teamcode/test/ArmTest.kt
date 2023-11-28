package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad

@TeleOp(name = "Arm Test")
@Suppress("unused")
class ArmTest : LinearOpMode() {
    lateinit var spindleDrive: DcMotor
    private var currentGamepad = Gamepad()
    private var pastGamepad = Gamepad()

    override fun runOpMode() {
        this.initialize()
        this.waitForStart()
        while (this.opModeIsActive()) {
            try {
                pastGamepad.copy(currentGamepad)
                currentGamepad.copy(gamepad1)
            } catch (ignored: Exception) {
            }

            spindleDrive.power = -currentGamepad.right_stick_y.toDouble() * 0.5
        }
    }

    private fun initialize() {
        this.spindleDrive = this.hardwareMap.dcMotor["spindleDrive"]
    }
}