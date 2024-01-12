//package org.firstinspires.ftc.teamcode.roadrunner
//
//import com.acmerobotics.roadrunner.geometry.Pose2d
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
//
//@TeleOp(name = "rrtest")
//class RoadrunnerTest : LinearOpMode() {
//    override fun runOpMode() {
//        val drive = SampleMecanumDrive(hardwareMap)
//        val myTrajectory = drive.trajectoryBuilder(Pose2d())
//            .strafeRight(10.0)
//            .strafeLeft(10.0)
//            .build()
//        waitForStart()
//        if (isStopRequested) return
//        drive.followTrajectory(myTrajectory)
//    }
//}
