package org.firstinspires.ftc.teamcode.modular

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.absoluteValue

class AutonomousRobot(private val telemetry: Telemetry) {
    private lateinit var leftFront: DcMotor
    private lateinit var rightFront: DcMotor
    private lateinit var leftBack: DcMotor
    private lateinit var rightBack: DcMotor
    private lateinit var drivetrainMotors: Array<DcMotor>
    private lateinit var imu: IMU
    private var currentGamepad1 = Gamepad()
    private var pastGamepad1 = Gamepad()
    private var currentGamepad2 = Gamepad()
    private var pastGamepad2 = Gamepad()
    private val registeredBooleanInputs =
        HashMap<AutonomousRobot.BooleanButton, (Robot.BooleanState) -> Unit>()
    lateinit var direction: RobotDirection
    private lateinit var encoder: DcMotor
    private lateinit var forwardDistance: DistanceSensor
    private lateinit var leftgrabber: ServoWrapper
    private lateinit var rightgrabber: ServoWrapper

    fun initialize(hardwareMap: HardwareMap) {
        this.leftFront = hardwareMap.dcMotor["lfdrive"]
        this.rightFront = hardwareMap.dcMotor["rfdrive"]
        this.leftBack = hardwareMap.dcMotor["lbdrive"]
        this.rightBack = hardwareMap.dcMotor["rbdrive"]
        this.imu = hardwareMap.get("imu") as IMU
        this.encoder = hardwareMap.get("encoder") as DcMotor
        this.forwardDistance = hardwareMap["fdistance"] as DistanceSensor
        this.leftgrabber = ServoWrapper(hardwareMap.servo["leftgrabber"], 0.0, 0.4)
        this.rightgrabber = ServoWrapper(hardwareMap.servo["rightgrabber"], 1.0, 0.55)
        this.drivetrainMotors = arrayOf(leftFront, rightFront, leftBack, rightBack)
        this.drivetrainMotors.forEach { it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER }
        leftFront.direction = DcMotorSimple.Direction.REVERSE
        rightFront.direction = DcMotorSimple.Direction.FORWARD
        leftBack.direction = DcMotorSimple.Direction.REVERSE
        rightBack.direction = DcMotorSimple.Direction.FORWARD
        this.imu.initialize(
            IMU.Parameters(
                RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
            )
        )
        this.imu.resetYaw()
        this.direction = RobotDirection.RED_FAR
        this.leftgrabber.set(0.5)
        Thread.sleep(500)
        this.rightgrabber.set(0.421)
    }

    private fun getHeading(): Float {
        return this.imu.getRobotOrientation(
            AxesReference.EXTRINSIC,
            AxesOrder.XZY,
            AngleUnit.DEGREES
        ).secondAngle
    }

    // headingOffset must be positive
    private fun turnUntilHeadingDelta(
        direction: TurnDirection,
        headingOffset: Double,
        speed: Double = 0.5
    ) {
        val firstHeading = this.getHeading()
        var lastHeading = firstHeading
        var change = 0.0
        while (change < headingOffset) {
            val currentHeading = getHeading()
            var delta = lastHeading - currentHeading
            if (currentHeading > 120 && lastHeading < -120) delta -= 360
            else if (currentHeading < -120 && lastHeading > 120) delta += 360
            change += delta.absoluteValue
            lastHeading = currentHeading
            this.turn(direction, speed)
        }
    }

    private fun moveUntilEncoder(target: Int, direction: Direction, speed: Double = 0.25) {
        this.encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        this.encoder.mode = DcMotor.RunMode.RUN_USING_ENCODER
        var mutableSpeed = speed
        if (direction == Direction.BACKWARDS) mutableSpeed *= -1
        val initialPosition = this.encoder.currentPosition
        var newPosition: Int
        while (true) {
            // panic
            if (forwardDistance.getDistance(DistanceUnit.INCH) < 5) break
            newPosition = (initialPosition - this.encoder.currentPosition).absoluteValue
            if (newPosition > target) break
            this.telemetry.addLine("new encoder: $newPosition")
            this.telemetry.addLine("init: $initialPosition")
            this.telemetry.update()
            this.drivetrainMotors.forEach { it.power = mutableSpeed }
        }
    }

    private fun turn(direction: TurnDirection, speed: Double = 0.2) {
        var mutableSpeed = speed
        if (direction == TurnDirection.LEFT) mutableSpeed *= -1
        this.leftFront.power = mutableSpeed
        this.rightFront.power = -mutableSpeed
        this.leftBack.power = mutableSpeed
        this.rightBack.power = -mutableSpeed
    }

    fun updateGamepads(gamepad1: Gamepad, gamepad2: Gamepad) {
        try {
            this.pastGamepad1.copy(this.currentGamepad1)
            this.pastGamepad2.copy(this.currentGamepad2)
            this.currentGamepad1.copy(gamepad1)
            this.currentGamepad2.copy(gamepad2)
        } catch (ignored: Exception) {
        }
    }

    private fun handleBooleanButtonTick(
        button: AutonomousRobot.BooleanButton,
        consumer: (Robot.BooleanState) -> Unit
    ) {
        val state = this.getState(button)
        consumer(state)
    }

    fun registerButton(button: AutonomousRobot.BooleanButton, consumer: () -> Unit) {
        val wrappingConsumer: (Robot.BooleanState) -> Unit = {
            if (it == Robot.BooleanState.RISING_EDGE) {
                consumer()
            }
        }

        this.registeredBooleanInputs[button] = wrappingConsumer
    }

    private fun getState(button: AutonomousRobot.BooleanButton): Robot.BooleanState {
        if (button.get() && !button.getPrev()) return Robot.BooleanState.RISING_EDGE
        else if (!button.get() && button.getPrev()) return Robot.BooleanState.FALLING_EDGE
        return Robot.BooleanState.INVARIANT
    }

    fun handleButtons() {
        this.registeredBooleanInputs.forEach { (button: AutonomousRobot.BooleanButton, consumer: (Robot.BooleanState) -> Unit) ->
            this.handleBooleanButtonTick(button, consumer)
        }
    }

    fun updateTelemetry() {
        this.telemetry.addLine(this.direction.name)
        this.telemetry.addLine("encoder: ${encoder.currentPosition}")
        this.telemetry.addLine("distance: ${forwardDistance.getDistance(DistanceUnit.INCH)}")
    }

    // mirror == blue
    fun near() {
        this.moveUntilEncoder(8000, Direction.FORWARD)
        this.turnUntilHeadingDelta(
            if (direction.color == Color.RED) TurnDirection.RIGHT else TurnDirection.LEFT,
            83.0
        )
        this.moveUntilEncoder(80000, Direction.FORWARD)
        this.leftgrabber.set(Robot.ServoDualState.OPEN)
        this.rightgrabber.set(Robot.ServoDualState.OPEN)
        this.moveUntilEncoder(2000, Direction.BACKWARDS)
    }

    fun far() {
        this.moveUntilEncoder(8000, Direction.FORWARD)
        this.turnUntilHeadingDelta(
            if (direction.color == Color.RED) TurnDirection.RIGHT else TurnDirection.LEFT,
            83.0
        )
        this.moveUntilEncoder(150000, Direction.FORWARD)
        this.leftgrabber.set(Robot.ServoDualState.OPEN)
        this.rightgrabber.set(Robot.ServoDualState.OPEN)
        this.moveUntilEncoder(2000, Direction.BACKWARDS)
    }

    inner class BooleanButton(private val button: BooleanButtonType) {
        fun get(): Boolean {
            return this.button.get(this@AutonomousRobot.currentGamepad1)
        }

        fun getPrev(): Boolean {
            return this.button.get(this@AutonomousRobot.pastGamepad1)
        }
    }

    enum class RobotDirection(val color: Color, val distance: Distance) {
        RED_FAR(Color.RED, Distance.FAR),
        RED_NEAR(Color.RED, Distance.NEAR),
        BLUE_FAR(Color.BLUE, Distance.FAR),
        BLUE_NEAR(Color.BLUE, Distance.NEAR)
    }

    enum class Color {
        RED, BLUE
    }

    enum class Distance {
        NEAR, FAR
    }

    enum class Direction {
        FORWARD, BACKWARDS
    }

    enum class TurnDirection {
        LEFT, RIGHT
    }

    inner class ServoWrapper(
        private val servo: Servo,
        private val open: Double,
        private val closed: Double,
        private var state: Robot.ServoDualState = Robot.ServoDualState.CLOSED
    ) {
        fun set(state: Robot.ServoDualState) {
            this.state = state
            when (state) {
                Robot.ServoDualState.OPEN -> this.servo.position = this.open
                Robot.ServoDualState.CLOSED -> this.servo.position = this.closed
            }
        }

        fun set(position: Double) {
            this.servo.position = position
        }
    }

}
