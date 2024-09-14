package org.firstinspires.ftc.teamcode.beepbeep;

import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kd_heading;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kd_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kd_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Ki_heading;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Ki_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Ki_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kp_heading;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kp_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kp_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kA;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kA_ang;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS_ang;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kV;
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
import org.firstinspires.ftc.teamcode.beepbeeplib.util.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pointfollow.BezierCurve;

@Config
@TeleOp(group = "dev")
public class FollowCombinedTest extends LinearOpMode {

    // x for bezier means y for linear :)

    public static double desired_x = 40;
    public static double desired_y = 0;
    public static double desired_heading = 0;

    // 0, 1, 30, 30
    public static double p1X = 0.0;
    public static double p2X = 0.0;
    public static double p3X = 15;
    public static double p4X = 30.0;

    // 0, 24, 1, 30
    public static double p1Y = 40.0;
    public static double p2Y = 70;
    public static double p3Y = 40;
    public static double p4Y = 55;

    public static double desired_heading2 = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        // Variables for PID control
        PIDController pid_x = new PIDController(Kp_x, Ki_x, Kd_x);
        PIDController pid_y = new PIDController(Kp_y, Ki_y, Kd_y);
        PIDController pid_heading = new PIDController(Kp_heading, Ki_heading, Kd_heading);

        double control_signal_x, control_signal_y, control_signal_heading = 0;
        double instantTargetPosition, u = 0;
        double instantTargetVelocity, duds, dxdu, dydu, x_vel, y_vel;
        double instantTargetAcceleration, du2ds, dx2du, dy2du, x_accel, y_accel;
        double totalY = 0, totalX = 0;

        double path_distance = Math.sqrt(Math.pow(desired_x, 2) + Math.pow(desired_y, 2));
        double path_angle = Math.atan2(desired_y, desired_x);

        BezierCurveCalc bezier_calc = new BezierCurveCalc();
        BezierCurve bezier_y = new BezierCurve(p1X, p2X, p3X, p4X);
        BezierCurve bezier_x = new BezierCurve(p1Y, p2Y, p3Y, p4Y);
        double curve_length = bezier_calc.bezier_length(bezier_x, bezier_y);

        MotionProfile motionProfile = new MotionProfile(maxAccel, maxVel, curve_length+path_distance);
        MotionProfile motionProfileAng = new MotionProfile(maxAngAccel, maxAngVel, Math.toRadians(desired_heading2));

        waitForStart();

        if (isStopRequested()) return;
        ElapsedTime timer = new ElapsedTime();
        timer.time();

        int pathMode = 1;

        while (opModeIsActive() && !isStopRequested()) {
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Angle
            double instantAngPosition = motionProfileAng.getPosition(timer.time());
            double angVel = motionProfileAng.getVelocity(timer.time());
            double angAccel = motionProfileAng.getAcceleration(timer.time());

            double powAng = angVel * kV_ang + angAccel * kA_ang;
            powAng = powAng + kS_ang * powAng/Math.abs(powAng);

            instantTargetPosition = motionProfile.getPosition(timer.time());
            instantTargetVelocity = motionProfile.getVelocity(timer.time());
            instantTargetAcceleration = motionProfile.getAcceleration(timer.time());

            if (instantTargetPosition < path_distance) {
                double xTargetPos = instantTargetPosition * Math.cos(path_angle);
                double xTargetVel = instantTargetVelocity * Math.cos(path_angle);
                double xTargetAccel = instantTargetAcceleration * Math.cos(path_angle);

                double yTargetPos = instantTargetPosition * Math.sin(path_angle);
                double yTargetVel = instantTargetVelocity * Math.sin(path_angle);
                double yTargetAccel = instantTargetAcceleration * Math.sin(path_angle);

                totalX = (xTargetVel * kV + xTargetAccel * kA);
                totalY = (yTargetVel * kV + yTargetAccel * kA);

                totalX = totalX + kS * totalX/Math.abs(totalX);
                totalY = totalY + kS * totalY/Math.abs(totalY);

                totalX = pid_x.calculate(xTargetPos, poseEstimate.getX()) + totalX;
                totalY = pid_y.calculate(yTargetPos, poseEstimate.getY()) + totalY;

                if (Double.isNaN(totalX)) {
                    totalX = 0;
                }

                if (Double.isNaN(totalY)) {
                    totalY = 0;
                }

//                control_signal_heading = pid_heading.calculate(angleWrap(Math.toRadians(desired_heading)), angleWrap(poseEstimate.getHeading())) + powAng;
            }
            if (instantTargetPosition >= path_distance) {
                // move to bezier curve cuz linear is finished
                // Position
                u = bezier_calc.bezier_param_of_disp(instantTargetPosition-path_distance, bezier_calc.get_sums(), bezier_calc.get_upsilon());
                control_signal_x = pid_x.calculate(bezier_x.bezier_get(u), poseEstimate.getX());
                control_signal_y = pid_y.calculate(bezier_y.bezier_get(u), poseEstimate.getY());

                // Velocity
                // s′(t)u′(s(t))x′(u(s(t)))
                duds = bezier_calc.bezier_param_of_disp_deriv(bezier_x, bezier_y, u);
                dxdu = bezier_x.bezier_deriv(u);
                dydu = bezier_y.bezier_deriv(u);
                x_vel = dxdu * duds * instantTargetVelocity;
                y_vel = dydu * duds * instantTargetVelocity;

                // Acceleration
                // (s'(t))^2 * (u'(s(t)))^2 * x''(u(s(t))) + ((s'(t))^2 * u''(s(t)) + s''(t) * u'(s(t))) * x'(u(s(t)))
                du2ds = bezier_calc.bezier_param_of_disp_deriv2(bezier_x, bezier_y, u);
                dx2du = bezier_x.bezier_deriv2(u);
                dy2du = bezier_y.bezier_deriv2(u);
                x_accel = Math.pow(instantTargetVelocity, 2) * Math.pow(duds, 2) * dx2du + Math.pow(instantTargetVelocity, 2) * du2ds + instantTargetAcceleration * duds * dxdu;
                y_accel = Math.pow(instantTargetVelocity, 2) * Math.pow(duds, 2) * dy2du + Math.pow(instantTargetVelocity, 2) * du2ds + instantTargetAcceleration * duds * dydu;

                // Combined
                totalX = control_signal_x + x_vel * kV_x + x_accel * kS_x;
                totalY = control_signal_y + y_vel * kV_y + y_accel * kS_y;
                control_signal_heading = pid_heading.calculate(angleWrap(Math.toRadians(desired_heading2)), angleWrap(poseEstimate.getHeading())) + powAng;
            }

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
            telemetry.addData("totalX", totalX);
            telemetry.addData("totalY", totalY);

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("x error", bezier_x.bezier_get(u) - poseEstimate.getX());

            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("y error", bezier_y.bezier_get(u) - poseEstimate.getY());

            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("control signal heading", control_signal_heading);
            telemetry.addData("heading error", desired_heading2 - poseEstimate.getHeading());

            telemetry.addData("instantTargetPosition", instantTargetPosition);
            telemetry.addData("instantTargetVelocity", instantTargetVelocity);
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