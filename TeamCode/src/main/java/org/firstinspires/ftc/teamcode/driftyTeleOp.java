package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class driftyTeleOp extends OpMode {
    private DcMotorEx lb1, lb2;
    private DcMotorEx rb1, rb2;
    private Servo steer;
    public enum Mode {LINEAR, QUINTIC, BEZIER, SIGMOID, SMOOTH};
    private Mode ymode = TeleOpConfig.ymode;
    private Mode rxMode = TeleOpConfig.rxMode;
    private SlewRateLimiter leftSlew = new SlewRateLimiter(TeleOpConfig.SlEW_A, TeleOpConfig.SlEW_D);
    private SlewRateLimiter rightSlew = new SlewRateLimiter(TeleOpConfig.SlEW_A, TeleOpConfig.SlEW_D);

    @Override
    public void init() {
        lb1 = hardwareMap.get(DcMotorEx.class, "lb1");
        lb2 = hardwareMap.get(DcMotorEx.class, "lb2");
        rb1 = hardwareMap.get(DcMotorEx.class, "rb1");
        rb2 = hardwareMap.get(DcMotorEx.class, "rb2");
        steer = hardwareMap.get(Servo.class, "turn");

        lb1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); //FLOAT --weird with slewratelimiter
        lb2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); //FLOAT
        rb1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); //FLOAT
        rb2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); //FLOAT

        lb1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lb2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rb1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rb2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lb1.setDirection(DcMotorEx.Direction.REVERSE);
        lb2.setDirection(DcMotorEx.Direction.REVERSE);
        rb1.setDirection(DcMotorEx.Direction.FORWARD);
        rb2.setDirection(DcMotorEx.Direction.FORWARD);
    }
    @Override
    public void loop() {
        Drive();
    }
    private void Drive() {
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        y = shape(y,ymode);
        rx = shape(rx,rxMode);

        double lbTarget = y + rx;
        double rbTarget = y - rx;

        double d = Math.max(Math.abs(y) + Math.abs(rx), 1);
        lbTarget /= d;
        rbTarget /= d;

        double lbPower = leftSlew.calculate(lbTarget);
        double rbPower = rightSlew.calculate(rbTarget);

        lb1.setPower(lbPower);
        lb2.setPower(lbPower);
        rb1.setPower(rbPower);
        rb2.setPower(rbPower);

        double steeringPosition = 0.5 + (rx * 0.3); //adjust 0.3 multiplier for steer pose
        steer.setPosition(Range.clip(steeringPosition, 0.0, 1.0));
    }
    private double shape(double i, Mode m)  {
        double abs = Math.abs(i);
        double SIGMOID_K = TeleOpConfig.SIGMOID_K;
        double BEZIER_P1 = TeleOpConfig.BEZIER_P1;
        if (abs < TeleOpConfig.STICK_DB) return 0;
        double r = (abs - TeleOpConfig.STICK_DB) / (1 - TeleOpConfig.STICK_DB);
        double o = 0;
        switch(m) {
            case LINEAR: o = r; break;
            case QUINTIC: //5th order quintic perlin
                o = r * r * r * (r * (r * 6 - 15) + 10);
                break;
            case SMOOTH: o = r * r * (3 - 2 * r); break;
            case SIGMOID: // norm log curve
                double rawSig = 1.0 / (1.0 + Math.exp(-SIGMOID_K * (r - 0.5)));
                double low = 1.0 / (1.0 + Math.exp(SIGMOID_K * 0.5));
                double high = 1.0 / (1.0 + Math.exp(-SIGMOID_K * 0.5));
                o = (rawSig - low) / (high - low);
                break;
            case BEZIER: //quadd bezier
                o = 2 * (1 - r) * r * BEZIER_P1 + Math.pow(r, 2);
                break;
        }
        return Math.copySign(o, i);
    }

    public class SlewRateLimiter {
        private double a; //accelLimit
        private double d; //decelLimit
        private double lastOutput = 0;
        private ElapsedTime timer;

        public SlewRateLimiter(double a, double d) {
            this.a = a;
            this.d = d;
            this.timer = new ElapsedTime();
            this.timer.reset();
        }

        public double calculate(double target) {
            double dt = timer.seconds();
            timer.reset();

            double step;
            //check if (decel OR change dir)
            if (Math.abs(target) < Math.abs(lastOutput) || Math.signum(target) != Math.signum(lastOutput)) {
                step = d * dt;
            } else {
                step = a * dt;
            }

            double error = target - lastOutput;

            //clamp the change
            if (Math.abs(error) > step) {
                lastOutput += Math.copySign(step, error);
            } else {
                lastOutput = target;
            }

            return lastOutput;
        }
    }
}
