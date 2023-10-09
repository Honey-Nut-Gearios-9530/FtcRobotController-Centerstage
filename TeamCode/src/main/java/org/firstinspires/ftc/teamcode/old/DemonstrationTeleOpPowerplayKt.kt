package org.firstinspires.ftc.teamcode.old

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF
import kotlin.math.absoluteValue

@TeleOp(name = "Demonstration TeleOp 2023 Kotlin", group = "demonstrations")
@Suppress("unused")
//@Disabled
class DemonstrationTeleOpPowerplayKt : LinearOpMode() {
    private val speedMultiplier = 1
    private lateinit var leftFront: DcMotor
    private lateinit var rightFront: DcMotor
    private lateinit var leftBack: DcMotor
    private lateinit var rightBack: DcMotor
    private var motors = arrayOf(leftFront, rightFront, leftBack, rightBack)

    override fun runOpMode() {
        initialize()
        waitForStart()
        // clear telemetry, telemetry::clearAll doesn't work
        telemetry.addLine()
        telemetry.update()

        while (opModeIsActive()) {
            // calculate amount of rotation
            val rotate = gamepad2.left_trigger + -gamepad2.right_trigger

            // matrix that hold the directions the wheels need to go
            // for movement in the x, y and rotational axes
            val directionMatrix = GeneralMatrixF(4, 3, floatArrayOf(
                    1f, -1f, -1f,
                    1f, 1f, 1f,
                    1f, 1f, -1f,
                    1f, -1f, 1f
            ))

            // matrix that hold the state of controller input that is translated to wheel speeds
            val inputMatrix = GeneralMatrixF(3, 1, floatArrayOf(
                    -gamepad1.right_stick_y,
                    -gamepad2.right_stick_x,
                    rotate
            ))

            // 4 length float that holds the speeds for each wheel
            // calculated by summing the weights of the wheel movement
            // for each axis via matrix multiplication
            val wheelSpeeds = (directionMatrix.multiplied(inputMatrix) as GeneralMatrixF).data

            // normalize the wheel speeds to within 0..1 and apply the new speeds
            val maxSpeed = wheelSpeeds.maxOfOrNull { it.absoluteValue }!!
            wheelSpeeds.map { it / maxSpeed }
                    .zip(motors)
                    .forEach { it.second.power = it.first.toDouble() * speedMultiplier }

            telemetry.addLine("Stopping")
            telemetry.update()
        }
    }

    private fun initialize() {
        // hardware devices
        // either hardwareMap::get and cast result to proper type
        // or pass [type].class as first parameter
        leftFront = hardwareMap.dcMotor["lfdrive"]
        rightFront = hardwareMap.dcMotor["rfdrive"]
        leftBack = hardwareMap.dcMotor["lbdrive"]
        rightBack = hardwareMap.dcMotor["rbdrive"]

        // faster than using encoders
        motors.forEach { it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER }

        // slows down faster when directional input is let off
        motors.forEach { it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE }

        // go forward on all pos inputs
        leftFront.direction = DcMotorSimple.Direction.REVERSE
        rightFront.direction = DcMotorSimple.Direction.FORWARD
        leftBack.direction = DcMotorSimple.Direction.REVERSE
        rightBack.direction = DcMotorSimple.Direction.FORWARD

        telemetry.addLine("Initialized devices")
        telemetry.update()
    }
}
