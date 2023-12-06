package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF
import kotlin.math.absoluteValue
import kotlin.reflect.KMutableProperty1

@TeleOp(name = "Night of the Arts Demonstration", group = "demonstrations")
@Suppress("unused")
// @Disabled
class NightoftheArts : LinearOpMode() {
    private var speedMultiplier = 0.6f
    private lateinit var drivetrainMotors: Array<DcMotor>
    private lateinit var armMotor: DcMotor
    private var currentGamepad = Gamepad()
    private var pastGamepad = Gamepad()

    override fun runOpMode() {
        initialize()
        waitForStart()
        // clear telemetry, telemetry::clearAll doesn't work
        telemetry.addLine()
        telemetry.update()

        while (opModeIsActive()) {
            try {
                pastGamepad.copy(currentGamepad)
                currentGamepad.copy(gamepad1)
            } catch (ignored: Exception) {
            }

            // if (risingEdge(Gamepad::a)) speedMultiplier *= -1

            // calculate amount of rotation
            val rotate = (gamepad1.left_trigger + -gamepad1.right_trigger) * speedMultiplier

            this.updateDrivetrain(-gamepad1.right_stick_y, -gamepad1.right_stick_x, rotate * 2)

            this.updateArm(-gamepad1.left_stick_y * 0.5f)

        }

        telemetry.addLine("Stopping")
        telemetry.update()
    }

    private fun updateArm(power: Float) {
        this.armMotor.power = power.toDouble()
    }

    private fun updateDrivetrain(y: Float, x: Float, rotate: Float) {
        // matrix that holds the directions the wheels need to go
        // for movement in the x, y and rotational axes
        val directionMatrix = GeneralMatrixF(
            4, 3, floatArrayOf(
                1f, -1f, -1f,
                1f, 1f, 1f,
                1f, 1f, -1f,
                1f, -1f, 1f
            )
        )

        // matrix that hold the state of controller input that is translated to wheel speeds
        val inputMatrix = GeneralMatrixF(
            3, 1, floatArrayOf(
                y, x, rotate
            )
        )

        // 4 length float that holds the speeds for each wheel
        // calculated by summing the weights of the wheel movement
        // for each axis via matrix multiplication
        val wheelSpeeds = (directionMatrix.multiplied(inputMatrix) as GeneralMatrixF).data

        // normalize the wheel speeds to within 0..1 and apply the new speeds
        val maxSpeed = wheelSpeeds.maxOfOrNull { it.absoluteValue }!!
        wheelSpeeds.map { if (maxSpeed > 1) it / maxSpeed else it }
            .zip(drivetrainMotors)
            .forEach { it.second.power = it.first.toDouble() * speedMultiplier }
    }

    private fun initialize() {
        // hardware devices
        // either hardwareMap::get and cast result to proper type
        // or pass [type].class as first parameter
        val leftFront = hardwareMap.dcMotor["lfdrive"]
        val rightFront = hardwareMap.dcMotor["rfdrive"]
        val leftBack = hardwareMap.dcMotor["lbdrive"]
        val rightBack = hardwareMap.dcMotor["rbdrive"]
        this.armMotor = hardwareMap.dcMotor["armbasedrive"]

        drivetrainMotors = arrayOf(leftFront, rightFront, leftBack, rightBack)

        // faster than using encoders
        drivetrainMotors.forEach { it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER }

        // slows down faster when directional input is let off
        drivetrainMotors.forEach { it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE }

        // go forward on all pos inputs
        leftFront.direction = DcMotorSimple.Direction.REVERSE
        rightFront.direction = DcMotorSimple.Direction.FORWARD
        leftBack.direction = DcMotorSimple.Direction.REVERSE
        rightBack.direction = DcMotorSimple.Direction.FORWARD

        telemetry.addLine("Initialized devices")
        telemetry.update()
    }

    private fun risingEdge(button: KMutableProperty1<Gamepad, Boolean>): Boolean {
        return button.get(this.gamepad1) && !button.get(this.gamepad2)
    }
}
