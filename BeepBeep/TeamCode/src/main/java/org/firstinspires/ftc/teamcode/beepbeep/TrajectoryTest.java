package org.firstinspires.ftc.teamcode.beepbeep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.beepbeeplib.util.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pointfollow.BezierCurve;

@Config
@TeleOp(group = "dev")
public class TrajectoryTest extends LinearOpMode {

//    // 0, 1, 30, 30
//    public static double p1X = 0.0;
//    public static double p2X = 0.0;
//    public static double p3X = 30.0;
//    public static double p4X = 30.0;
//
//    // 0, 24, 1, 30
//    public static double p1Y = 0.0;
//    public static double p2Y = 60.0;
//    public static double p3Y = 0.0;
//    public static double p4Y = 60.0;

    // 0, 1, 30, 30
    public static double p1X = 0.0;
    public static double p2X = 1.0;
    public static double p3X = 30.0;
    public static double p4X = 30.0;

    // 0, 24, 1, 30
    public static double p1Y = 0.0;
    public static double p2Y = 24.0;
    public static double p3Y = 1.0;
    public static double p4Y = 30.0;

    public static double desired_heading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        Trajectory traj = new Trajectory(drive, new BezierCurve(p1X, p2X, p3X, p4X), new BezierCurve(p1Y, p2Y, p3Y, p4Y), desired_heading);

        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();

        if (isStopRequested()) return;

        ElapsedTime timer = new ElapsedTime();
        timer.time();

        while (opModeIsActive() && !isStopRequested()) {
            Pose2d poseEstimate = drive.getPoseEstimate();

            traj.followBezier(timer.time());

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }

}


/*
Trajectory path1 = new Trajectory(new Bezier(points), heading))
path1.start()

 */