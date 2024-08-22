package org.firstinspires.ftc.teamcode.drive.opmode.TeleOp;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Match-TeleOp")

public class Fortnite_God extends LinearOpMode {


    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor Intake = null;

    private Servo left_servo_slide;





    private CRServo wheel_bucket;

    private Servo left_servo_lift;

    private Servo right_servo_lift;

    private Servo right_servo_slide;

    private DcMotor slide = null;
    private Servo drone = null;

    private DcMotor left_lift = null;

    private DcMotor right_lift = null;



    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront"); //backright, port 2
        Intake = hardwareMap.get(DcMotor.class, "Intake");  //Intake
        slide = hardwareMap.get(DcMotor.class, "slide");
        left_lift = hardwareMap.get(DcMotor.class, "left_lift");
        right_lift =  hardwareMap.get(DcMotor.class, "right_lift");


        wheel_bucket = hardwareMap.get(CRServo.class, "wheel_bucket"); // Port 5 Expansion Hub
        right_servo_slide = hardwareMap.get(Servo.class, "right_servo_slide");
        left_servo_slide = hardwareMap.get(Servo.class, "left_servo_slide");// port 4 Expansion Hub
        drone = hardwareMap.get(Servo.class, "drone");

        left_servo_lift = hardwareMap.get(Servo.class, "left_servo_lift");
        right_servo_lift = hardwareMap.get(Servo.class, "right_servo_lift");


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.REVERSE);




        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        double spower = .9;


        waitForStart();
        //bucket.setPosition(0.85);
        drone.setPosition(.52);

        while (opModeIsActive()) {


            double G1rightStickY = gamepad1.right_stick_y;
            double G1leftStickY = gamepad1.left_stick_y;
            double G1rightStickX = gamepad1.right_stick_x;
            double G1leftStickX = gamepad1.left_stick_x;





            double power = 0.7;

            while (!isStopRequested()) {
                robot.setWeightedDrivePower(
                        new Pose2d(
                                (-gamepad1.left_stick_y) * power,
                                (-gamepad1.left_stick_x) * power,
                                (-gamepad1.right_stick_x) * power

                        )
                );


                boolean G1rightBumper = gamepad1.right_bumper;
                boolean G1leftBumper = gamepad1.left_bumper;
                boolean G1UD = gamepad1.dpad_up;   // up dpad
                boolean G1DD = gamepad1.dpad_down; //Down dpad
                boolean G1RD = gamepad1.dpad_right;// right dpad
                boolean G1LD = gamepad1.dpad_left; //left dpad
                boolean G1Y = gamepad1.y;
                boolean G1B = gamepad1.b;
                boolean G1X = gamepad1.x;
                boolean G1A = gamepad1.a;
                double G1RT = gamepad1.right_trigger;
                double G1LT = gamepad1.left_trigger;
                //Second controller (Intake/linear slide)
                double G2leftStickY = gamepad2.left_stick_y;
                boolean G2B = gamepad2.b;
                boolean G2Y = gamepad2.y;
                boolean G2A = gamepad2.a;
                boolean G2X = gamepad2.x;
                boolean G2UD = gamepad2.dpad_up; // up dpad
                boolean G2DD = gamepad2.dpad_down; // down dpad
                boolean G2RD = gamepad2.dpad_right;// right dpad
                boolean G2LD = gamepad2.dpad_left; //left dpad
                double G2LT = gamepad2.left_trigger;
                double G2RT = gamepad2.right_trigger;
                boolean G2rightBumper = gamepad2.right_bumper;
                boolean G2leftBumper = gamepad2.left_bumper;
                boolean G2back = gamepad2.back;
                /*
                if (G1UD) {
                    leftFront.setPower(spower);
                    leftRear.setPower(spower);
                    rightFront.setPower(spower);
                    rightRear.setPower(spower);
                } else {
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                }


                if (G1DD) {
                    leftFront.setPower(-spower);
                    leftRear.setPower(-spower);
                    rightFront.setPower(-spower);
                    rightRear.setPower(-spower);
                } else {
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                }


                if (G1RD) {
                    leftFront.setPower(-spower);
                    leftRear.setPower(spower);
                    rightFront.setPower(spower);
                    rightRear.setPower(-spower);
                } else {
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                }

                if (G1RD) {
                    leftFront.setPower(spower);
                    leftRear.setPower(-spower);
                    rightFront.setPower(-spower);
                    rightRear.setPower(spower);
                } else {
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                }

                 */



                if (G2UD) { // Intake Forward
                    Intake.setPower(-0.7);
                    wheel_bucket.setPower(-1);



                } else if (abs(gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) > 0) {
                    //right_servo_slide.setPosition(.4);




                } else if (G2DD) {

                    wheel_bucket.setPower(1);
                    Intake.setPower(.7);

                } else {
                    Intake.setPower(0);
                    wheel_bucket.setPower(0);
                }

                if (G1Y) {
                    right_servo_lift.setPosition(.4);
                    sleep(1000);
                    drone.setPosition(.9);
                    sleep(500);
                    drone.setPosition(.4);

                }


                if (G2RT > 0) { // Upward
                    slide.setPower(0.85);
                    right_servo_slide.setPosition(.65 );
                } else if (G2LT > 0) {
                    slide.setPower(-.85);
                    right_servo_slide.setPosition(.65);
                } else {
                    slide.setPower(.08);
                }

                if (G2Y) {
                    wheel_bucket.setPower(1);
                }


                if (G1RT > 0) {
                    left_lift.setPower(1);
                    right_lift.setPower(1);
                } else if (G1LT > 0) {
                    left_lift.setPower(-1);
                    right_lift.setPower(-1);
                } else {
                    left_lift.setPower(0);
                    right_lift.setPower(0);
                }
                if (G2leftBumper) {

                    left_servo_lift.setPosition(.35);
                    right_servo_lift.setPosition(.2);

                }else if (G1Y){
                    //right_servo_lift.setPosition(0.4);
                }else if (G2rightBumper) {
                    left_servo_lift.setPosition(.65);
                    right_servo_lift.setPosition(.7);

                } else if (G2X) {
                    right_servo_slide.setPosition(.18);     //Left Position not in play currently
                         //Nuetral == .25 (Right)
                                                            //intake == ?    (Right)
                } else if (G2B) {
                    right_servo_slide.setPosition(.75);

                }else if (G2Y){
                    wheel_bucket.setPower(1);
                }



                telemetry.addData("Status", "Running");
                telemetry.update();

            }






    }

}
}

