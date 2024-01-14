package org.firstinspires.ftc.teamcode.modular

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad

@Suppress("unused")
@TeleOp(name = "Robot Tester", group = "danger")
class RobotTester : OpMode() {
    private var robot = Robot(this.telemetry)

    // TODO: touch sensor for arm retracting so it can be controlled in auto / auto pose
    override fun init() {
        this.robot.initialize(this.hardwareMap)
        this.robot.registerButton(this.robot.BooleanButton(Gamepad::a, 0), Robot::switchDirection)
        this.robot.registerButton(
            this.robot.BooleanButton(Gamepad::b, 0),
            Robot::quarterSpeed
        )
        this.robot.registerButton(
            this.robot.BooleanButton(Gamepad::dpad_down, 1),
            Robot::pixelPickupPose
        )
        this.robot.registerButton(this.robot.BooleanButton(Gamepad::dpad_up, 0), Robot::launchDrone)
        this.robot.registerButton(this.robot.BooleanButton(Gamepad::left_bumper, 1)) { ->
            this.robot.toggleClaw(Robot.LRServo.LEFT)
        }
        this.robot.registerButton(this.robot.BooleanButton(Gamepad::right_bumper, 1)) { ->
            this.robot.toggleClaw(Robot.LRServo.RIGHT)
        }
        this.robot.registerButton(this.robot.BooleanButton(Gamepad::left_bumper, 0)) { ->
            this.telemetry.addLine("left")
            this.robot.toggleGrabber(Robot.LRServo.LEFT)
        }
        this.robot.registerButton(this.robot.BooleanButton(Gamepad::right_bumper, 0)) { ->
            this.robot.toggleGrabber(Robot.LRServo.RIGHT)
            this.telemetry.addLine("right")
        }
        this.robot.registerDrivetrainButtons(
            this.robot.FloatButton(Gamepad::right_stick_y, 0),
            this.robot.FloatButton(Gamepad::right_stick_x, 0),
            this.robot.FloatButton(Gamepad::left_trigger, 0),
            this.robot.FloatButton(Gamepad::right_trigger, 0)
        )
        this.robot.registerArmButtons(
            this.robot.FloatButton(Gamepad::left_stick_y, 1),
            this.robot.FloatButton(Gamepad::right_trigger, 1),
            this.robot.FloatButton(Gamepad::left_trigger, 1),
            this.robot.FloatButton(Gamepad::right_stick_y, 1),
            this.robot.FloatButton(Gamepad::right_stick_x, 1)
        )
    }

    override fun loop() {
        this.robot.tick(this.gamepad1, this.gamepad2)
        this.telemetry.update()
    }
}
