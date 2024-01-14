package org.firstinspires.ftc.teamcode.modular;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Gamepad

@Autonomous(name = "Autonomous Tester", group = "Prod", preselectTeleOp = "Robot Tester")
class AutonomousTester : LinearOpMode() {
    private var robot = AutonomousRobot(this.telemetry)

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
        while (!opModeIsActive()) {
            this.robot.updateGamepads(this.gamepad1, this.gamepad2)
            this.robot.handleButtons()
            this.robot.updateTelemetry()
            this.telemetry.update()
        }
        this.resetRuntime()
        when (this.robot.direction.distance) {
            AutonomousRobot.Distance.NEAR -> this.robot.near()
            AutonomousRobot.Distance.FAR -> this.robot.far()
        }
    }
}
