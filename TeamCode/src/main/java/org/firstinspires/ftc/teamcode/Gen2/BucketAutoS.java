package org.firstinspires.ftc.teamcode.Gen2;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="BucketAutoS")
@Disabled

public class BucketAutoS extends LinearOpMode {
    public DcMotor slidesR = null;
    public DcMotor slidesL = null;
    public DcMotor horizontalExtension = null;
    public DcMotor motorMTConverter = null;

    public Servo extensionWrist = null;
    public Servo bucketArm = null;
    public Servo bucketWrist = null;
    public Servo turret = null;
    public Servo MTConverter = null;
    public Servo specimenArm = null;
    public Servo specimenWrist = null;
    public Servo specimenClaw = null;

    public CRServo intake = null;

    public int SlidesPosition = 0;

    public double WristPosition = 0;
    public double Position =.5;
    public double HEPosition =.1;
    public double SPEED = .5;

    double[] timeArray = new double[20];

    double deltaTime = 0;

    public static boolean timerInitted = false;
    public static boolean timerInitted2 = false;
    public static boolean timerInitted3 = false;

    private final int TEMP_DOWN = 640;

    public static double x, y, finalAngle;

    ElapsedTime currentTime;



    @Override
    public void runOpMode() throws InterruptedException {

        robotHardwarePinPoint robot = new robotHardwarePinPoint(hardwareMap);

        robot.resetDriveEncoders();
        robot.odo.resetPosAndIMU();

        currentTime = new ElapsedTime();

        intake = hardwareMap.crservo.get("intake");
        extensionWrist = hardwareMap.servo.get("extensionWrist");

        bucketWrist = hardwareMap.servo.get("bucketWrist");
        bucketArm = hardwareMap.servo.get("bucketArm");
        turret = hardwareMap.servo.get("turret");
        MTConverter = hardwareMap.servo.get("MTConverter");
        specimenArm = hardwareMap.servo.get("specimenArm");
        specimenClaw = hardwareMap.servo.get("specimenClaw");
        specimenWrist = hardwareMap.servo.get("specimenWrist");

        slidesR = hardwareMap.dcMotor.get("slidesR");
        slidesL = hardwareMap.dcMotor.get("slidesL");
        motorMTConverter = hardwareMap.dcMotor.get("motorMTConverter");
        horizontalExtension = hardwareMap.dcMotor.get("horizontalExtension");

        horizontalExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorMTConverter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        horizontalExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorMTConverter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //slidesR.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesL.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontalExtension.setDirection(DcMotorSimple.Direction.REVERSE);

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        //intake.setPower(Position);...........

        /*********************************************************************
         * INIT
         **********************************************************************
         */
        while (!isStarted() && !isStopRequested()) {
            extensionWrist.setPosition(0);  //Tuned
            turret.setPosition(robot.TURRET_LEFT); //Tuned
            intake.setPower(0);  //Tuned
            bucketWrist.setPosition(1);  //Tuned
            bucketArm.setPosition(.99);  //Tuned
            //MTConverter.setPosition(1);  //Tuned
            specimenArm.setPosition(.1);
            specimenClaw.setPosition(1);
            specimenWrist.setPosition(.65);

            SlidesPosition = (int) robot.SLIDE_INIT;
            slidesL.setTargetPosition(SlidesPosition);
            slidesL.setPower(.2);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesR.setTargetPosition(SlidesPosition);
            slidesR.setPower(.2);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            horizontalExtension.setTargetPosition(0);
            horizontalExtension.setPower(.5);
            horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);  //Tuned

            //motorMTConverter.setTargetPosition(0);
            //motorMTConverter.setPower(.2);
            //motorMTConverter.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            robot.changeAccuracy(1, Math.toRadians(1));
            robot.changeSpeed(1, 1);

        }

        /**
         * Calling the Methods
         */
        firstDump(robot);
        firstCycle(robot);
        secondCycle(robot);
        thirdCycle(robot);
        park(robot);




    }
    /**
     * Methods
     */

    public void firstDump(robotHardwarePinPoint robot) {

        /**
         *
         * First sample being dumped
         *
         */

        robot.goToPos(0, 5,0,Math.toRadians(90));

        robot.changeAccuracy(.5,Math.toRadians(1));

        //double x = -15,y = 7, finalAngle = Math.toRadians(45);
        double x = -19;
        double y = 3;
        double finalAngle = Math.toRadians(45);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(180));


            slidesR.setTargetPosition(2200);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(2200);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.mecanumDrive(0,0,0,0);

        bucketArm.setPosition(robot.BUCKET_ARM_DROP);//dropping
        bucketWrist.setPosition(robot.BUCKET_WRIST_DROP);//drop

        robot.wait(850, robot.odometers);
    }

    public void firstCycle(robotHardwarePinPoint robot){


        /**
         *
         * End of first placement
         *
         *
         * Start of going to the second sample
         *
         */

        bucketArm.setPosition(robot.BUCKET_ARM_REST);//going to rest position/down
        bucketWrist.setPosition(robot.BUCKET_WRIST_REST);//rest

        robot.wait(500, robot.odometers);

        x = 5;
        y = 25;
        finalAngle = Math.toRadians(180);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(-90));

            //Slides Down
            slidesR.setTargetPosition(100);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(100);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }

        robot.goToPos(5,35,Math.toRadians(180),Math.toRadians(0));

        extensionWrist.setPosition(robot.WRIST_PICKUP);
        turret.setPosition(robot.TURRET_LEFT);

        robot.mecanumDrive(0,0,0,0);
        robot.wait(250, robot.odometers);

        intake.setPower(-1);

        robot.wait(350, robot.odometers);

        horizontalExtension.setTargetPosition(100);   //EXTEND
        horizontalExtension.setPower(1);
        horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(500, robot.odometers);

        horizontalExtension.setTargetPosition(250);   //RETRACT
        horizontalExtension.setPower(1);
        horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(500, robot.odometers);

        extensionWrist.setPosition(robot.WRIST_DROP);

        robot.wait(525, robot.odometers);

        intake.setPower(1);

        robot.wait(750, robot.odometers);

        extensionWrist.setPosition(robot.WRIST_PICKUP);
        intake.setPower(0);

        robot.wait(250, robot.odometers);

        x = -19; y = 3; finalAngle = Math.toRadians(45); //Change to dropoff pos

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(180));


            slidesR.setTargetPosition(2200);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(2200);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.mecanumDrive(0,0,0,0);

        bucketArm.setPosition(robot.BUCKET_ARM_DROP);//dropping
        bucketWrist.setPosition(robot.BUCKET_WRIST_DROP);//drop

        robot.wait(750, robot.odometers);

        bucketArm.setPosition(robot.BUCKET_ARM_REST);//going to rest position/down
        bucketWrist.setPosition(robot.BUCKET_WRIST_REST);//rest

        robot.wait(500, robot.odometers);
    }

    public void secondCycle(robotHardwarePinPoint robot){

        /**
         *
         *
         * going to the third sample / second pick  up
         *
         *
         */

        x = -4;
        y = 34;
        finalAngle = Math.toRadians(180);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(-90));


            slidesR.setTargetPosition(100);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(100);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.mecanumDrive(0,0,0,0);
        robot.wait(250, robot.odometers);

        extensionWrist.setPosition(robot.WRIST_PICKUP);
        intake.setPower(-1);

        robot.wait(350, robot.odometers);

        horizontalExtension.setTargetPosition(100);   //EXTEND
        horizontalExtension.setPower(1);
        horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(500, robot.odometers);

        horizontalExtension.setTargetPosition(250);   //RETRACT
        horizontalExtension.setPower(1);
        horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(500, robot.odometers);

        extensionWrist.setPosition(robot.WRIST_DROP);

        robot.wait(525, robot.odometers);

        intake.setPower(1);

        robot.wait(750, robot.odometers);

        extensionWrist.setPosition(robot.WRIST_PICKUP);
        intake.setPower(0);

        robot.wait(250, robot.odometers);

        x = -18;
        y = 3;
        finalAngle = Math.toRadians(45);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(180));


            slidesR.setTargetPosition(2200);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(2200);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.mecanumDrive(0,0,0,0);

        bucketArm.setPosition(robot.BUCKET_ARM_DROP);//dropping
        bucketWrist.setPosition(robot.BUCKET_WRIST_DROP);//drop

        robot.wait(750, robot.odometers);

        bucketArm.setPosition(robot.BUCKET_ARM_REST);//going to rest position/down
        bucketWrist.setPosition(robot.BUCKET_WRIST_REST);//rest

        robot.wait(500, robot.odometers);
    }

    public void thirdCycle(robotHardwarePinPoint robot){

        /**
         *
         *
         * prep for fourth sample
         *
         *
         */

        extensionWrist.setPosition(.5);

        //going to the third pick up
        x = -13;
        y = 34;
        finalAngle = Math.toRadians(180);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(-90));


            slidesR.setTargetPosition(100);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(100);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }
        robot.mecanumDrive(0,0,0,0);
        robot.wait(250, robot.odometers);

        extensionWrist.setPosition(robot.WRIST_PICKUP);
        intake.setPower(-1);

        robot.wait(350, robot.odometers);

        horizontalExtension.setTargetPosition(100);   //EXTEND
        horizontalExtension.setPower(1);
        horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(500, robot.odometers);

        horizontalExtension.setTargetPosition(250);   //RETRACT
        horizontalExtension.setPower(1);
        horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(500, robot.odometers);

        extensionWrist.setPosition(robot.WRIST_DROP);

        robot.wait(525, robot.odometers);

        intake.setPower(1);

        robot.wait(750, robot.odometers);

        extensionWrist.setPosition(robot.WRIST_PICKUP);
        intake.setPower(0);

        robot.wait(250, robot.odometers);

        x = -18;
        y = 3;
        finalAngle = Math.toRadians(45);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(180));


            slidesR.setTargetPosition(2200);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(2200);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.mecanumDrive(0,0,0,0);

        bucketArm.setPosition(robot.BUCKET_ARM_DROP);//dropping
        bucketWrist.setPosition(robot.BUCKET_WRIST_DROP);//drop

        robot.wait(750, robot.odometers);

        bucketArm.setPosition(robot.BUCKET_ARM_REST);//going to rest position/down
        bucketWrist.setPosition(robot.BUCKET_WRIST_REST);//rest

        robot.wait(500, robot.odometers);

        slidesR.setTargetPosition(0);
        slidesR.setPower(1);
        slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesL.setTargetPosition(0);
        slidesL.setPower(1);
        slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(1000, robot.odometers);
    }

    public void park(robotHardwarePinPoint robot){

        x = 0;y = 50; finalAngle = Math.toRadians(0);
        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {
            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(180));
            robot.wait(1500,robot.odometers);
            //Reset to Init Pos. for TeleOp
            bucketWrist.setPosition(1);  //Tuned
            bucketArm.setPosition(.99);  //Tuned
            MTConverter.setPosition(1);  //Tuned
            specimenArm.setPosition(.1);
            specimenClaw.setPosition(1);
            specimenWrist.setPosition(.65);

            horizontalExtension.setTargetPosition(0);
            horizontalExtension.setPower(.5);
            horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);  //Tuned   Smaller # is Out

        }
        robot.mecanumDrive(0,0,0,.6);

        robot.wait(1000,robot.odometers);

        robot.mecanumDrive(0,0,0,.6);
    }



}