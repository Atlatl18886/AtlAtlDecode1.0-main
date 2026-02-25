package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TeleOpConfig {
    public static double STICK_DB = 0.05;
    public static double SlEW_A = 4.0;
    public static double SlEW_D = 6.5;
    public static driftyTeleOp.Mode ymode = driftyTeleOp.Mode.BEZIER;
    public static driftyTeleOp.Mode rxMode = driftyTeleOp.Mode.QUINTIC;

    //preset-sepcific
    public static double SIGMOID_K = 10;
    public static double BEZIER_P1 = 0.25;

}
