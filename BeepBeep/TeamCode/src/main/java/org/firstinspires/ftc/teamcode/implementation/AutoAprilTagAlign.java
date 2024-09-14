package org.firstinspires.ftc.teamcode.implementation;

import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.*;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.beepbeep.MotionProfile;
import org.firstinspires.ftc.teamcode.beepbeep.PIDController;
import org.firstinspires.ftc.teamcode.beepbeeplib.util.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class AutoAprilTagAlign extends LinearOpMode {

    public static int tagID = 5;
    public static int targetDist = 15;

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

        double path_distance = 0;
        double path_angle = 0;
        MotionProfile motionProfile = null;

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "backWebcam"))
                .setCameraResolution(new Size(960,720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {}

        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(255); // change value

        double desired_x = 0, desired_y = 0, desired_heading = 0;
        boolean goToTag = false;

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            if (tagProcessor.getDetections().size() > 0) {
                for (AprilTagDetection tag : tagProcessor.getDetections()) {
                    if (tag.id == tagID) {
                        desired_y = tag.ftcPose.x;
                        desired_x = -1 * tag.ftcPose.y + targetDist;
                        desired_heading = angleWrap(-1 * tag.ftcPose.yaw);

                        telemetry.addData("x", tag.ftcPose.x);
                        telemetry.addData("y", tag.ftcPose.y);
//                        telemetry.addData("z", tag.ftcPose.z);
//                        telemetry.addData("roll", tag.ftcPose.roll);
//                        telemetry.addData("pitch", tag.ftcPose.pitch);
                        telemetry.addData("yaw", tag.ftcPose.yaw);
                        telemetry.update();
                    }
                }
            }

            if (gamepad1.a && !goToTag) {
                while (gamepad1.a) {}
//                path_distance = Math.sqrt(Math.pow(desired_x, 2) + Math.pow(desired_y, 2));
//                path_angle = Math.atan2(desired_y, desired_x); // check
//                motionProfile = new MotionProfile(maxAccel, maxVel, path_distance);

                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

                goToTag = true;
                timer.reset();
                timer.time();
            }

            if (goToTag) {
                double motionMultiplier = 1;

                double instantTargetPosition = motionProfile.getPosition(timer.time());
                double vel = motionProfile.getVelocity(timer.time());
                double accel = motionProfile.getAcceleration(timer.time());

                if (instantTargetPosition >= path_distance) {
                    goToTag = false;
                    drive.setMotorPowers(0, 0, 0, 0);
                    break;
                }

                double xTargetPos = instantTargetPosition * Math.cos(path_angle);
                double xTargetVel = vel * Math.cos(path_angle);
                double xTargetAccel = accel * Math.cos(path_angle);

                double yTargetPos = instantTargetPosition * Math.sin(path_angle);
                double yTargetVel = vel * Math.sin(path_angle);
                double yTargetAccel = accel * Math.sin(path_angle);

                double powX = motionMultiplier * (xTargetVel * kV + xTargetAccel * kA);
                double powY = motionMultiplier * (yTargetVel * kV + yTargetAccel * kA);

                powX = powX + kS * powX/Math.abs(powX);
                powY = powY + kS * powY/Math.abs(powY);

                if (Double.isNaN(powX)) {
                    powX = 0;
                    xTargetPos = desired_x;
                }

                if (Double.isNaN(powY)) {
                    powY = 0;
                    yTargetPos = desired_y;
                }

                Pose2d poseEstimate = drive.getPoseEstimate();

                control_signal_x = pid_x.calculate(xTargetPos, poseEstimate.getX()) + powX;
                control_signal_y = pid_y.calculate(yTargetPos, poseEstimate.getY()) + powY;
                control_signal_heading = pid_heading.calculate(angleWrap(desired_heading), angleWrap(poseEstimate.getHeading()));

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
            }

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
