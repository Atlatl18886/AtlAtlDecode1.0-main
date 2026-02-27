package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Config.TeleOpConfig;
import org.firstinspires.ftc.teamcode.helpers.Curve;
import org.firstinspires.ftc.teamcode.helpers.SlewRateLimiter;

@TeleOp(name="main tele", group ="Main")
public class TeleOpMain extends OpMode {
    private DcMotorEx lb, rb, lf, rf;
    private Servo abc123abc;
    private final SlewRateLimiter yLimit = new SlewRateLimiter(), 
                                   xLimit = new SlewRateLimiter(), 
                                   rxLimit = new SlewRateLimiter();

    @Override
    public void init() {
        lb = hardwareMap.get(DcMotorEx.class, "leftBack");
        lf = hardwareMap.get(DcMotorEx.class, "leftFront");
        rb = hardwareMap.get(DcMotorEx.class, "rightBack");
        rf = hardwareMap.get(DcMotorEx.class, "rightFront");
        abc123abc = hardwareMap.get(Servo.class, "THEservo");

        lb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lb.setDirection(DcMotorEx.Direction.REVERSE);
        lf.setDirection(DcMotorEx.Direction.REVERSE);
        rb.setDirection(DcMotorEx.Direction.FORWARD);
        rf.setDirection(DcMotorEx.Direction.FORWARD);
    }

    @Override
    public void loop() {
        Drive();
    }

    private void Drive() {
        double y = deadband(gamepad1.left_stick_y);
        double x = deadband(-gamepad1.left_stick_x) * 1.1;
        double rx = deadband(-gamepad1.right_stick_x);

        y = TeleOpConfig.DRIVE_CURVE.apply(y, TeleOpConfig.BEZIER_P1, TeleOpConfig.BEZIER_P2, TeleOpConfig.EXP_A);
        x = TeleOpConfig.DRIVE_CURVE.apply(x, TeleOpConfig.BEZIER_P1, TeleOpConfig.BEZIER_P2, TeleOpConfig.EXP_A);
        rx = TeleOpConfig.DRIVE_CURVE.apply(rx, TeleOpConfig.BEZIER_P1, TeleOpConfig.BEZIER_P2, TeleOpConfig.EXP_A);

        if (gamepad1.right_stick_button) rx *= TeleOpConfig.AIM_TURN_SCALE;
        if (gamepad1.left_stick_button) {
            y *= TeleOpConfig.AIM_TURN_SCALE + 0.1;
            x *= TeleOpConfig.AIM_TURN_SCALE + 0.1;
        }

        if (TeleOpConfig.USE_SLEW) {
            y = yLimit.calculate(y, TeleOpConfig.Y_ACCEL, TeleOpConfig.Y_DECEL);
            x = xLimit.calculate(x, TeleOpConfig.X_ACCEL, TeleOpConfig.X_DECEL);
            rx = rxLimit.calculate(rx, TeleOpConfig.RX_ACCEL, TeleOpConfig.RX_DECEL);
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        lf.setPower((y + x + rx) / denominator);
        lb.setPower((y - x + rx) / denominator);
        rf.setPower((y - x - rx) / denominator);
        rb.setPower((y + x - rx) / denominator);
    }

    private double deadband(double input) {
        if (Math.abs(input) < TeleOpConfig.STICK_DB) return 0;
        return (input - Math.signum(input) * TeleOpConfig.STICK_DB) / (1 - TeleOpConfig.STICK_DB);
    }
}
