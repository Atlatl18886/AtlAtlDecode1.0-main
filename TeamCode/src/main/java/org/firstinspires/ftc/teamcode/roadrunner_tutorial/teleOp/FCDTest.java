package org.firstinspires.ftc.teamcode.roadrunner_tutorial.teleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Disabled

@TeleOp(name = "FCD Test")
public class FCDTest extends OpMode {

    Deadline gamepadRateLimit = new Deadline(250, TimeUnit.MILLISECONDS);
    DcMotor leftFront, rightFront, rightBack, leftBack;
    IMU imu;
    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");


        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
    }

    @Override
    public void loop() {
        FSDDrive();
    }

    private void FSDDrive() {
        double vertical = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        //max power oover here is 0.95
        double drivePower = 0.95 - (0.6 * gamepad1.right_trigger);

        if (gamepadRateLimit.hasExpired() && gamepad1.a) {
            imu.resetYaw();
            gamepadRateLimit.reset();
        }

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double adjustedVertical = vertical * Math.cos(heading) - strafe * Math.sin(heading);
        double adjustedStrafe = vertical * Math.sin(heading) + strafe * Math.cos(heading);



        double max = Math.max(Math.abs(adjustedStrafe) + Math.abs(adjustedVertical) + Math.abs(turn), 1);

        double RFPower = ((turn + (adjustedVertical - adjustedStrafe)) / max) * drivePower;
        double RBPower = ((turn + (adjustedVertical + adjustedStrafe)) / max) * drivePower;
        double LFPower = (((-turn) + (adjustedVertical + adjustedStrafe)) / max) * drivePower;
        double LBPower = (((-turn) + (adjustedVertical - adjustedStrafe)) / max) * drivePower;

        rightFront.setPower(RFPower);
        rightBack.setPower(RBPower);
        leftFront.setPower(LFPower);
        leftBack.setPower(LBPower);
    }
}