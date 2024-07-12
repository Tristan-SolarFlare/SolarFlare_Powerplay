package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36.86, 61.64, Math.toRadians(-90.00)))
                        .strafeTo(new Vector2d(-36.66, 36.05))
                        .strafeToLinearHeading(new Vector2d(-57.17, 36.46),Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(-57.17, 8.43), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(51.89, 8.43), 0)
                        .strafeToLinearHeading(new Vector2d(51.89, 37.27), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(51.89, 8.43), 0)
                .strafeToLinearHeading(new Vector2d(-57.17, 8.43), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(51.89, 8.43), 0)
                .strafeToLinearHeading(new Vector2d(51.89, 37.27), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(51.89, 8.43), 0)
                .strafeToLinearHeading(new Vector2d(-57.17, 8.43), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(51.89, 8.43), 0)
                .strafeToLinearHeading(new Vector2d(51.89, 37.27), Math.toRadians(0))
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}