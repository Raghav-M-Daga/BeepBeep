package org.firstinspires.ftc.teamcode.pointfollow;

import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kd_heading;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kd_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kd_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Ki_heading;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Ki_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Ki_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kp_heading;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kp_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kp_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kA_ang;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS_ang;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kV_ang;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kV_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kV_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.maxAccel;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.maxAngAccel;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.maxAngVel;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.maxVel;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants;
import org.firstinspires.ftc.teamcode.beepbeep.BezierCurveCalc;
import org.firstinspires.ftc.teamcode.beepbeep.MotionProfile;
import org.firstinspires.ftc.teamcode.beepbeep.PIDController;
import org.firstinspires.ftc.teamcode.beepbeeplib.Drive;
import org.firstinspires.ftc.teamcode.beepbeeplib.util.SampleMecanumDrive;

@Config
@TeleOp(group = "dev")
public class FollowCurvePointTest extends LinearOpMode {

    // 0, 1, 30, 30
    public static double p1X = 0.0;
    public static double p2X = 0.0;
    public static double p3X = 15;
    public static double p4X = 30.0;

    // 0, 24, 1, 300
    public static double p1Y = 0.0;
    public static double p2Y = 30;
    public static double p3Y = 0;
    public static double p4Y = 15;

    public static double desired_heading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        // Variables for PID control
        PIDController pid_x = new PIDController(Kp_x, Ki_x, Kd_x);
        PIDController pid_y = new PIDController(Kp_y, Ki_y, Kd_y);
        PIDController pid_heading = new PIDController(Kp_heading, Ki_heading, Kd_heading);

        double control_signal_x = 0, control_signal_y = 0, control_signal_heading;
        double instantTargetPosition, u;
        double instantTargetVelocity, duds, dxdu, dydu, x_vel = 0, y_vel = 0;
        double instantTargetAcceleration, du2ds, dx2du, dy2du, x_accel = 0, y_accel = 0;
        double totalY, totalX;

//        BezierCurveCalc bezier_calc = new BezierCurveCalc();
        BezierCurve bezier_y = new BezierCurve(p1X, p2X, p3X, p4X);
        BezierCurve bezier_x = new BezierCurve(p1Y, p2Y, p3Y, p4Y);

        CurvePointCalc calc = new CurvePointCalc(bezier_x, bezier_y);
//        double curve_length = bezier_calc.bezier_length(bezier_x, bezier_y);

//        MotionProfile motionProfile = new MotionProfile(maxAccel, maxVel, curve_length);
//        MotionProfile motionProfileAng = new MotionProfile(maxAngAccel, maxAngVel, Math.toRadians(desired_heading));

        telemetry.addData("x", 0);
        telemetry.addData("y", 0);
        telemetry.addData("heading", 0);
        telemetry.addData("target x", 0);
        telemetry.addData("target y", 0);
        telemetry.addData("target x vel", 0);
        telemetry.addData("target y vel", 0);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;
        ElapsedTime timer = new ElapsedTime();
        timer.time();

        while (opModeIsActive() && !isStopRequested()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d poseVector = new Vector2d(poseEstimate.getX(), poseEstimate.getY());

            // Angle
//            double instantAngPosition = motionProfileAng.getPosition(timer.time());
//            double angVel = motionProfileAng.getVelocity(timer.time());
//            double angAccel = motionProfileAng.getAcceleration(timer.time());

//            double powAng = angVel * kV_ang + angAccel * kA_ang;
//            powAng = powAng + kS_ang * powAng/Math.abs(powAng);

            Vector2d instantTargetPose = calc.closest_point(poseVector);

            control_signal_x = pid_x.calculate(instantTargetPose.getX(), poseEstimate.getX());
            control_signal_y = pid_y.calculate(instantTargetPose.getY(), poseEstimate.getY());

            double feedforwardAngle = calc.angle_tangent(poseVector);
            x_vel = maxVel * Math.cos(feedforwardAngle);
            y_vel = maxVel * Math.sin(feedforwardAngle);

            totalX = control_signal_x; //  + x_vel * kV_x + x_accel * kS_x
            totalY = control_signal_y; //  + y_vel * kV_y + y_accel * kS_y
            control_signal_heading = pid_heading.calculate(angleWrap(Math.toRadians(desired_heading)), angleWrap(poseEstimate.getHeading()));

            if (Double.isNaN(control_signal_heading)) {
                control_signal_heading = 0;
            }

            Vector2d input = new Vector2d(
                    totalX,
                    totalY
            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            control_signal_heading
                    )
            );

            drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("target x", instantTargetPose.getX());
            telemetry.addData("target y", instantTargetPose.getY());
            telemetry.addData("target x vel", x_vel);
            telemetry.addData("target y vel", y_vel);
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