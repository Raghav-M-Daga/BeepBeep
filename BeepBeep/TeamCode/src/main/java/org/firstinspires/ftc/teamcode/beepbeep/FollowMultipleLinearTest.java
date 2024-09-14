package org.firstinspires.ftc.teamcode.beepbeep;

import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.*;

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

@Config
@TeleOp(group = "dev")
public class FollowMultipleLinearTest extends LinearOpMode {

    public static double desired_x1 = 40;
    public static double desired_y1 = 40;
    public static double desired_heading1 = 0;

    public static double desired_x2 = 40;
    public static double desired_y2 = 40;
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

        double control_signal_x = 0;
        double control_signal_y = 0;
        double control_signal_heading = 0;

        double path_distance1 = Math.sqrt(Math.pow(desired_x1, 2) + Math.pow(desired_y1, 2));
        double path_distance2 = Math.sqrt(Math.pow(desired_x2-desired_x1, 2) + Math.pow(desired_y2-desired_y1, 2));
        double path_distance = path_distance1 + path_distance2;

        double path_angle1 = Math.atan2(desired_y1, desired_x1);
        double path_angle2 = Math.atan2(desired_y2-desired_y1, desired_x2-desired_x1);

        MotionProfile motionProfile = new MotionProfile(maxAccel, maxVel, path_distance);

        waitForStart();

        if (isStopRequested()) return;
        ElapsedTime timer = new ElapsedTime();
        timer.time();

        double modeAngle = path_angle1;
        double modeDesiredHeading = desired_heading1;

        while (opModeIsActive() && !isStopRequested()) {
            double motionMultiplier = 1;

            double instantTargetPosition = motionProfile.getPosition(timer.time());
            double vel = motionProfile.getVelocity(timer.time());
            double accel = motionProfile.getAcceleration(timer.time());

            if (instantTargetPosition >= path_distance1) {
                modeAngle = path_angle2;
                modeDesiredHeading = desired_heading2;
            }

            double xTargetPos = instantTargetPosition * Math.cos(modeAngle);
            double xTargetVel = vel * Math.cos(modeAngle);
            double xTargetAccel = accel * Math.cos(modeAngle);

            double yTargetPos = instantTargetPosition * Math.sin(modeAngle);
            double yTargetVel = vel * Math.sin(modeAngle);
            double yTargetAccel = accel * Math.sin(modeAngle);

            double powX = motionMultiplier * (xTargetVel * kV + xTargetAccel * kA);
            double powY = motionMultiplier * (yTargetVel * kV + yTargetAccel * kA);

            powX = powX + kS * powX/Math.abs(powX);
            powY = powY + kS * powY/Math.abs(powY);

            if (Double.isNaN(powX)) {
                powX = 0;
                xTargetPos = desired_x2;
            }

            if (Double.isNaN(powY)) {
                powY = 0;
                yTargetPos = desired_y2;
            }

            Pose2d poseEstimate = drive.getPoseEstimate();

            control_signal_x = pid_x.calculate(xTargetPos, poseEstimate.getX()) + powX;
            control_signal_y = pid_y.calculate(yTargetPos, poseEstimate.getY()) + powY;
            control_signal_heading = pid_heading.calculate(angleWrap(Math.toRadians(modeDesiredHeading)), angleWrap(poseEstimate.getHeading()));

            Vector2d input = new Vector2d(
                    control_signal_x,
                    control_signal_y
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
            telemetry.addData("x error", xTargetPos - poseEstimate.getX());
            telemetry.addData("y error", yTargetPos - poseEstimate.getY());
            telemetry.addData("heading error", modeDesiredHeading - poseEstimate.getHeading());
            telemetry.addData("path position", instantTargetPosition);
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