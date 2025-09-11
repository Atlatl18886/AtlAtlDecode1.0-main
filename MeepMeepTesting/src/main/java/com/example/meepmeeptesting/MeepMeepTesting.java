package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
//                .lineToX(30)
//                .turn(Math.toRadians(90))
//                .lineToY(30)
//                .turn(Math.toRadians(90))
//                .lineToX(0)
//                .strafeToLinearHeading(new Vector2d(-50,-30),Math.toRadians(90),new TranslationalVelConstraint(90))
//                .waitSeconds(2)
//                .splineToSplineHeading(new Pose2d(20,20,Math.toRadians(90)),Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(30,30,Math.toRadians(180)),Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(0,60,Math.toRadians(270)),Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-30,30,Math.toRadians(0)),Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(0,0,Math.toRadians(0)),Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}