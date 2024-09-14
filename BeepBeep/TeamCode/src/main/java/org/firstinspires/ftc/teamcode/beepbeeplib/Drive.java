package org.firstinspires.ftc.teamcode.beepbeeplib;

import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kd_heading;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kd_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kd_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Ki_heading;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Ki_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Ki_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kp_heading;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kp_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.Kp_y;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pointfollow.BezierCurve;
import org.firstinspires.ftc.teamcode.beepbeeplib.util.PIDController;
import org.firstinspires.ftc.teamcode.beepbeeplib.util.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.beepbeeplib.util.TrajFollower;

@Config
public class Drive extends SampleMecanumDrive {
    PIDController pid_x = new PIDController(Kp_x, Ki_x, Kd_x);
    PIDController pid_y = new PIDController(Kp_y, Ki_y, Kd_y);
    PIDController pid_heading = new PIDController(Kp_heading, Ki_heading, Kd_heading);
    public static double error = 1;

    public Drive(HardwareMap hwMap) {
        super(hwMap);
    }

    public void followTrajectory(BezierCurve bezier_x, BezierCurve bezier_y, double desired_heading) {
        TrajFollower follower = new TrajFollower(this);

        follower.followBezier(error, bezier_x, bezier_y, pid_x, pid_x, pid_heading, desired_heading);
    }
}
