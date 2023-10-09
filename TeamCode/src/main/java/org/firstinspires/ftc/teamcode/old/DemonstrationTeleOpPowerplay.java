package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;

@TeleOp(name = "Demonstration TeleOp 2023", group = "demonstrations")
@SuppressWarnings("unused")
@Disabled
public class DemonstrationTeleOpPowerplay extends LinearOpMode {
    double speedMultiplier = 0.6;
    DcMotor leftFront, rightFront, leftBack, rightBack;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        // clear telemetry, telemetry::clearAll doesn't work
        telemetry.addLine();
        telemetry.update();

        while (opModeIsActive()) {
            // calculate amount of rotation
            float rotate = gamepad2.right_trigger == 0 ? gamepad2.left_trigger : -gamepad2.right_trigger;

            // matrix that hold the directions the wheels need to go
            // for movement in the x, y and rotational axes
            GeneralMatrixF directionMatrix = new GeneralMatrixF(4, 3, new float[]{
                    1, -1, -1,
                    1,  1,  1,
                    1,  1, -1,
                    1, -1,  1
            });

            // matrix that hold the state of controller input that is translated to wheel speeds
            GeneralMatrixF inputMatrix = new GeneralMatrixF(3, 1, new float[]{
                    -gamepad1.right_stick_y,
                    -gamepad2.right_stick_x,
                    rotate
            });

            // 4 length float that holds the speeds for each wheel
            // calculated by summing the weights of the wheel movement
            // for each axis via matrix multiplication
            float[] wheelSpeeds = ((GeneralMatrixF) directionMatrix.multiplied(inputMatrix)).getData();

            // normalize the wheel speeds to within 0..1
            float maxSpeed = 1;
            for (float wheelSpeed : wheelSpeeds) {
                wheelSpeed = Math.abs(wheelSpeed);
                if (wheelSpeed > maxSpeed) {
                    maxSpeed = wheelSpeed;
                }
            }
            float[] normalizedWheelSpeeds = new float[wheelSpeeds.length];
            for (int i = 0; i < normalizedWheelSpeeds.length; i++) {
                normalizedWheelSpeeds[i] = wheelSpeeds[i] / maxSpeed;
            }

            leftFront.setPower(normalizedWheelSpeeds[0] * speedMultiplier);
            rightFront.setPower(normalizedWheelSpeeds[1] * speedMultiplier);
            leftBack.setPower(normalizedWheelSpeeds[2] * speedMultiplier);
            rightBack.setPower(normalizedWheelSpeeds[3] * speedMultiplier);
        }

        telemetry.addLine("Stopping");
        telemetry.update();
    }


    public void initialize() {
        // hardware devices
        // either hardwareMap::get and cast result to proper type
        // or pass [type].class as first parameter
        leftFront = hardwareMap.dcMotor.get("lfdrive");
        rightFront = hardwareMap.dcMotor.get("rfdrive");
        leftBack = hardwareMap.dcMotor.get("lbdrive");
        rightBack = hardwareMap.dcMotor.get("rbdrive");

        // faster than using encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // slows down faster when directional input is let off
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // go forward on all pos inputs
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Initialized devices");
        telemetry.update();
    }
}
