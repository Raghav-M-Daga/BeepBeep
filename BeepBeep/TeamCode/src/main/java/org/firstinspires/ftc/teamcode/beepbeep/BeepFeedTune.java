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
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kV_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kV_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.maxAccel;
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

import java.util.Objects;

@Config
@TeleOp(group = "dev")
public class BeepFeedTune extends LinearOpMode {

    // Target positions and heading
    public static double desired_x = 60;
    public static double desired_y = 0;
    public static double desired_heading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        double x_dist = desired_x;

        double control_signal_x = 0;
        double control_signal_y = 0;
        double control_signal_heading = 0;

        double powX = 0, powY = 0;

        double path_distance = Math.sqrt(Math.pow(desired_x, 2) + Math.pow(desired_y, 2));
        double path_angle = Math.atan2(desired_y, desired_x);
        MotionProfile motionProfile = new MotionProfile(maxAccel, maxVel, path_distance);

        int val = 1;
        int counter = 0;

        waitForStart();

        if (isStopRequested()) return;
        ElapsedTime timer = new ElapsedTime();
        timer.time();

        while (opModeIsActive() && !isStopRequested()) {
            if (motionProfile.isFinished(timer.time())) {
                counter++;
                val *= -1;

                desired_x = desired_x * val;
                path_angle = Math.atan2(desired_y, desired_x);
                motionProfile = new MotionProfile(maxAccel, maxVel, path_distance);

                timer.reset();
            }

            double instantTargetPosition = motionProfile.getPosition(timer.time());

//            if (motionProfile.isFinished(timer.time())) {
//                instantTargetPosition = path_distance;
//            }
            double vel = motionProfile.getVelocity(timer.time());
            double accel = motionProfile.getAcceleration(timer.time());

            double xTargetPos = instantTargetPosition * Math.cos(path_angle);
            double xTargetVel = vel * Math.cos(path_angle);
            double xTargetAccel = accel * Math.cos(path_angle);

            double yTargetPos = instantTargetPosition * Math.sin(path_angle);
            double yTargetVel = vel * Math.sin(path_angle);
            double yTargetAccel = accel * Math.sin(path_angle);

            powX = (xTargetVel * kV_x + xTargetAccel * kA);
            powY = (yTargetVel * kV_y + yTargetAccel * kA);

            powX = powX + kS * powX/Math.abs(powX);
            powY = powY + kS * powY/Math.abs(powY);
            telemetry.addData("# MP Finished", counter);

            Pose2d poseEstimate = drive.getPoseEstimate();

            control_signal_x = powX;
            control_signal_y = powY;
//            control_signal_heading = pid_heading.calculate(angleWrap(Math.toRadians(desired_heading)), angleWrap(poseEstimate.getHeading()));

            Vector2d input = new Vector2d(
                    control_signal_x,
                    0
            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            0
                    )
            );

            drive.update();

            // Print pose to telemetry
            Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");
            double currentVelo = poseVelo.getX();

            // update telemetry
            telemetry.addData("targetVelocity", val*motionProfile.getVelocity(timer.time()));
            telemetry.addData("measuredVelocity", currentVelo);
            telemetry.addData("error", val*motionProfile.getVelocity(timer.time()) - currentVelo);
            telemetry.addData("desired x", desired_x);
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