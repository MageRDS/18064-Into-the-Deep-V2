package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 14.75)
                .setDimensions(16.75, 17)
                .setStartPose(new Pose2d(-35, 60, Math.toRadians(90)))
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.84, -58.8, Math.toRadians(-90)))
                                //.splineToLinearHeading(new Pose2d(50, -9, Math.toRadians(-180)), Math.toRadians(20))
                                //.waitSeconds(1)
                                .lineTo(new Vector2d(-36.84,-32))
                                //.waitSeconds(1)
                                .splineToLinearHeading(new Pose2d(-50, -9, Math.toRadians(-180)), Math.toRadians(20))
                                //.waitSeconds(1)
                                .lineTo(new Vector2d(30,-10))
                                //.waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(50, -35.9, Math.toRadians(180)))
                                //.waitSeconds(1)
                                .lineTo(new Vector2d(47,-15))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}