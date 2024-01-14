package org.firstinspires.ftc.teamcode.modular


import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Gamepad

@Suppress("unused")
@Autonomous(name = "Autonomous Tester", group = "Prod", preselectTeleOp = "Robot Tester")
class AutonomousTester : LinearOpMode() {
    private var robot = AutonomousRobot(this.telemetry, this::isStopRequested, this::opModeIsActive)

    override fun runOpMode() {
        this.robot.initialize(this.hardwareMap)
        this.robot.registerButton(robot.BooleanButton(Gamepad::a)) { ->
            this.robot.direction = AutonomousRobot.RobotDirection.BLUE_FAR
        }
        this.robot.registerButton(robot.BooleanButton(Gamepad::x)) { ->
            this.robot.direction = AutonomousRobot.RobotDirection.BLUE_NEAR
        }
        this.robot.registerButton(robot.BooleanButton(Gamepad::y)) { ->
            this.robot.direction = AutonomousRobot.RobotDirection.RED_FAR
        }
        this.robot.registerButton(robot.BooleanButton(Gamepad::b)) { ->
            this.robot.direction = AutonomousRobot.RobotDirection.RED_NEAR
        }
        this.robot.registerButton(this.robot.BooleanButton(Gamepad::left_bumper)) { ->
            this.robot.toggleClaw(Robot.LRServo.RIGHT)
        }
        this.robot.registerButton(this.robot.BooleanButton(Gamepad::right_bumper)) { ->
            this.robot.toggleClaw(Robot.LRServo.LEFT)
        }
        this.robot.registerButton(this.robot.BooleanButton(Gamepad::dpad_up)) { ->
            this.robot.rightgrabber.set(Robot.ServoDualState.OPEN)
        }
        while (!isStarted && !isStopRequested) {
            this.robot.updateGamepads(this.gamepad1, this.gamepad2)
            this.robot.handleButtons()
            this.robot.updateTelemetry()
            this.robot.pitchWrist.position =
                (this.robot.pitchWrist.position + -gamepad2.left_stick_y * 0.002).coerceIn(0.0..1.0)
            this.telemetry.addLine(this.robot.pitchWrist.position.toString())
            this.telemetry.update()
        }
        if (isStarted) {
            this.resetRuntime()
            if (!this.robot.checkActive()) return
            this.robot.manipulatePixel()
            if (!this.robot.checkActive()) return
            this.robot.moveAndTurn();
            if (!this.robot.checkActive()) return
            when (this.robot.direction.distance) {
                AutonomousRobot.Distance.NEAR -> this.robot.moveUntilEncoder(
                    80000,
                    AutonomousRobot.Direction.FORWARD
                )

                AutonomousRobot.Distance.FAR -> this.robot.moveUntilEncoder(
                    150000,
                    AutonomousRobot.Direction.FORWARD
                )
            }
            if (!this.robot.checkActive()) return
            this.robot.scoreWithReverse()
        }
    }
}
