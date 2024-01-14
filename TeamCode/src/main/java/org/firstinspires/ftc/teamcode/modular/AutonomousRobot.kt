package org.firstinspires.ftc.teamcode.modular

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.absoluteValue
import kotlin.reflect.KFunction0

class AutonomousRobot(
    private val telemetry: Telemetry,
    private val stopped: KFunction0<Boolean>,
    private val active: KFunction0<Boolean>
) {
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
    lateinit var rightgrabber: ServoWrapper
    private lateinit var lclaw: ServoWrapper
    private lateinit var rclaw: ServoWrapper
    private lateinit var armBaseMotor: DcMotor
    private lateinit var armBottom: TouchSensor
    lateinit var pitchWrist: Servo
    private lateinit var rollWrist: Servo

    fun checkActive(): Boolean {
        val yes = active() && !stopped()
        telemetry.addLine(yes.toString())
        return yes
    }

    fun initialize(hardwareMap: HardwareMap) {
        this.leftFront = hardwareMap.dcMotor["lfdrive"]
        this.rightFront = hardwareMap.dcMotor["rfdrive"]
        this.leftBack = hardwareMap.dcMotor["lbdrive"]
        this.rightBack = hardwareMap.dcMotor["rbdrive"]
        this.imu = hardwareMap["imu"] as IMU
        this.encoder = hardwareMap["encoder"] as DcMotor
        this.forwardDistance = hardwareMap["fdistance"] as DistanceSensor
        this.leftgrabber = ServoWrapper(hardwareMap.servo["leftgrabber"], 0.0, 0.4)
        this.rightgrabber = ServoWrapper(hardwareMap.servo["rightgrabber"], 1.0, 0.428)
        this.lclaw = ServoWrapper(hardwareMap.servo["lclawservo"], 0.8, 0.5)
        this.rclaw = ServoWrapper(hardwareMap.servo["rclawservo"], 0.85, 0.5)
        this.armBottom = hardwareMap.touchSensor["armbottom"]
        this.armBaseMotor = hardwareMap.dcMotor["armbasedrive"]
        this.drivetrainMotors = arrayOf(leftFront, rightFront, leftBack, rightBack)
        this.pitchWrist = hardwareMap.servo["verticalwrist"]
        this.rollWrist = hardwareMap.servo["horizontalwrist"]
        this.drivetrainMotors.forEach { it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER }
        leftFront.direction = DcMotorSimple.Direction.REVERSE
        rightFront.direction = DcMotorSimple.Direction.FORWARD
        leftBack.direction = DcMotorSimple.Direction.REVERSE
        rightBack.direction = DcMotorSimple.Direction.FORWARD
        this.pixelPickupPose()
        // this.pitchWrist.position = 0.8461111 /* TODO: find offset such that arm down doesn't get stuck on pixels but can place them down and keep aligned with cowcatcher */
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
        // this.leftgrabber.set(0.5)
        // Thread.sleep(500)
        // this.rightgrabber.set(0.421)
        this.leftgrabber.set(Robot.ServoDualState.OPEN)
        this.rightgrabber.set(Robot.ServoDualState.OPEN)
        this.lclaw.set(Robot.ServoDualState.CLOSED)
        this.rclaw.set(Robot.ServoDualState.CLOSED)
        // this.leftgrabber.set(Robot.ServoDualState.CLOSED)
        // this.rightgrabber.set(Robot.ServoDualState.CLOSED)
    }

    fun pixelPickupPose() {
        this.pitchWrist.position = 0.8461111
        this.rollWrist.position = 0.39
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
        while (change < headingOffset && this.checkActive()) {
            val currentHeading = getHeading()
            var delta = lastHeading - currentHeading
            if (currentHeading > 120 && lastHeading < -120) delta -= 360
            else if (currentHeading < -120 && lastHeading > 120) delta += 360
            change += delta.absoluteValue
            lastHeading = currentHeading
            this.turn(direction, speed)
        }
    }

    fun moveUntilEncoder(target: Int, direction: Direction, speed: Double = 0.25) {
        this.encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        this.encoder.mode = DcMotor.RunMode.RUN_USING_ENCODER
        var mutableSpeed = speed
        if (direction == Direction.BACKWARDS) mutableSpeed *= -1
        val initialPosition = this.encoder.currentPosition
        var newPosition: Int
        while (this.checkActive()) {
            // panic
            if (forwardDistance.getDistance(DistanceUnit.INCH) < 4) break
            newPosition = (initialPosition - this.encoder.currentPosition).absoluteValue
            if (newPosition > target) break
            // this.telemetry.addLine("new encoder: $newPosition")
            // this.telemetry.addLine("init: $initialPosition")
            // this.telemetry.update()
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
    }

    private fun moveAndTurn() {
        this.moveUntilEncoder(8000, Direction.FORWARD)
        this.turnUntilHeadingDelta(
            if (direction.color == Color.RED) TurnDirection.RIGHT else TurnDirection.LEFT,
            83.0
        )
    }


    fun scoreWithReverse() {
        this.leftgrabber.set(Robot.ServoDualState.OPEN)
        this.rightgrabber.set(Robot.ServoDualState.OPEN)
        this.moveUntilEncoder(2000, Direction.BACKWARDS)
    }

    @Suppress("ControlFlowWithEmptyBody")
    fun manipulatePixel() {
        // done in init
        // this.leftgrabber.set(Robot.ServoDualState.OPEN)
        // this.rightgrabber.set(Robot.ServoDualState.OPEN)
        this.armBaseMotor.power = -0.5
        while (checkActive() && !this.armBottom.isPressed) {
            telemetry.addLine("running arm ${armBaseMotor.currentPosition}")
            telemetry.update()
        }
        this.armBaseMotor.power = 0.0
        this.lclaw.set(Robot.ServoDualState.CLOSED)
        this.rclaw.set(Robot.ServoDualState.CLOSED)
        val timedelta = System.currentTimeMillis()
        this.armBaseMotor.power = 0.5
        // normalizes if flop happens after dress or before
        while (checkActive() && this.armBottom.isPressed) {
        }
        while (checkActive() && (System.currentTimeMillis() - timedelta) < 500) {
        }
        this.armBaseMotor.power = 0.0
        this.leftgrabber.set(Robot.ServoDualState.CLOSED)
        this.rightgrabber.set(Robot.ServoDualState.CLOSED)
    }

    fun toggleClaw(claw: Robot.LRServo) {
        when (claw) {
            Robot.LRServo.LEFT -> this.lclaw.toggle()
            Robot.LRServo.RIGHT -> this.rclaw.toggle()
        }
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

        fun toggle() {
            this.set(this.state.opposite())
        }
    }

}
