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

import java.util.Objects;

@Config
@TeleOp(group = "dev")
public class BeepFeedforwardTuner extends LinearOpMode {

    // Target positions and heading
    public static double desired_x = 40;
    public static double desired_y = 0;
    public static double desired_heading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        Pose2d setPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(setPose);

        // Variables for PID control
//        PIDController pid_x = new PIDController(Kp_x, Ki_x, Kd_x);
//        PIDController pid_y = new PIDController(Kp_y, Ki_y, Kd_y);

        double control_signal_x = 0;
        double control_signal_y = 0;
        double control_signal_heading = 0;
        double powX = 0;
        double powY = 0;
        double val = 1;

        waitForStart();

        if (isStopRequested()) return;

        double path_distance = Math.sqrt(Math.pow(desired_x, 2) + Math.pow(desired_y, 2));
        double path_angle = Math.atan2(desired_y, desired_x * val);
        MotionProfile motionProfile = new MotionProfile(maxAccel, maxVel, path_distance);

        ElapsedTime timer = new ElapsedTime();
        timer.time();

        while (opModeIsActive() && !isStopRequested()) {
            double instantTargetPosition = motionProfile.getPosition(timer.time());
            double vel = motionProfile.getVelocity(timer.time());
            double accel = motionProfile.getAcceleration(timer.time());

            double xTargetPos = instantTargetPosition * Math.cos(path_angle);
            double xTargetVel = vel * Math.cos(path_angle);
            double xTargetAccel = accel * Math.cos(path_angle);

            double yTargetPos = instantTargetPosition * Math.sin(path_angle);
            double yTargetVel = vel * Math.sin(path_angle);
            double yTargetAccel = accel * Math.sin(path_angle);

            powX = (xTargetVel * kV_x + xTargetAccel * kA_x);
            powY = (yTargetVel * kV_y + yTargetAccel * kA_y);

            powX = (powX + kS_x*powX/Math.abs(powX));
            powY = (powY + kS_x*powY/Math.abs(powY));

            if (motionProfile.isFinished(timer.time())) {
                val = val * -1;
                desired_x = val * desired_x;
                path_distance = Math.sqrt(Math.pow(desired_x, 2) + Math.pow(desired_y, 2));
                path_angle = Math.atan2(desired_y, desired_x);
                motionProfile = new MotionProfile(maxAccel, maxVel, path_distance);

                timer.reset();
                timer.time();
            }

            Pose2d poseEstimate = drive.getPoseEstimate();

            control_signal_x = powX;
            control_signal_y = powY;
//            control_signal_heading = pid_heading.calculate((Math.toRadians(desired_heading)), (poseEstimate.getHeading()));

            Vector2d input = new Vector2d(
                    control_signal_x,
                    control_signal_y
            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            0
                    )
            );

            drive.update();

            Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");
            double currentVelo = poseVelo.getX();

            // update telemetry
            telemetry.addData("targetVelocity", val*motionProfile.getVelocity(timer.time()));
            telemetry.addData("measuredVelocity", currentVelo);
            telemetry.addData("error", val*motionProfile.getVelocity(timer.time()) - currentVelo);

            // Print pose to telemetry
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.addData("x error", xTargetPos - poseEstimate.getX());
//            telemetry.addData("y error", yTargetPos - poseEstimate.getY());
//            telemetry.addData("heading error", desired_heading - poseEstimate.getHeading());
//            telemetry.addData("path position", instantTargetPosition);
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