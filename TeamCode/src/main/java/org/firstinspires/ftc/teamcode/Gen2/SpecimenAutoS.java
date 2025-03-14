package org.firstinspires.ftc.teamcode.Gen2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="SpecimenAutoS")
//@Disabled

public class SpecimenAutoS extends LinearOpMode {
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

    public double WristPosition = 0;
    public int SlidesPosition = 0;

    public double Position =.5;
    public double HEPosition =.1;

    public double SPEED = .5;

    double[] timeArray = new double[20];

    public static boolean timerInitted = false;
    public static boolean timerInitted2 = false;
    public static boolean timerInitted3 = false;

    private final int TEMP_DOWN = 640;

    public static double x, y, finalAngle;

    ElapsedTime currentTime;

    double deltaTime = 0;

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


        while (!isStarted() && !isStopRequested()) {
            extensionWrist.setPosition(0);  //Tuned
            turret.setPosition(robot.TURRET_LEFT); //Tuned
            intake.setPower(0);  //Tuned
            bucketWrist.setPosition(1);  //Tuned
            bucketArm.setPosition(.99);  //Tuned
            MTConverter.setPosition(1);  //Tuned
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


        firstPlace(robot);

        firstPush(robot);

        secondPush(robot);

        //thirdPush(robot);

        firstCycle(robot, -7);

        firstCycle(robot, -14);

        firstCycle(robot, -15);

        park(robot);


    }

    public void firstPlace(robotHardwarePinPoint robot) {

        robot.changeAccuracy(1,Math.toRadians(1));
        robot.changeSpeed(1,1);

        x = 34;
        y = 18;
        finalAngle = Math.toRadians(0);


        slidesR.setTargetPosition(1250);
        slidesR.setPower(1);
        slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesL.setTargetPosition(1250);
        slidesL.setPower(1);
        slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        specimenArm.setPosition(robot.SPECIMEN_ARM_PLACE);
        specimenWrist.setPosition(robot.SPECIMEN_WRIST_PLACE);

        robot.wait(300,robot.odometers);

        deltaTime = currentTime.milliseconds() + 1600;

        while((Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) && deltaTime > currentTime.milliseconds()){

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(30));

        }

        telemetry.addData("time4", currentTime.milliseconds());
        telemetry.update();

        robot.mecanumDrive(0,0,0,0);

        robot.changeAccuracy(2,Math.toRadians(10));
        robot.changeSpeed(1,1);

        x = 18;
        y = 12;
        finalAngle = Math.toRadians(0);
        robot.goToPos(x,y,finalAngle,Math.toRadians(180));

        specimenArm.setPosition(robot.SPECIMEN_ARM_PICKUP);
        specimenWrist.setPosition(robot.SPECIMEN_WRIST_PICKUP);

    }

    public void firstPush(robotHardwarePinPoint robot) {

        slidesR.setTargetPosition(100);
        slidesR.setPower(1);
        slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesL.setTargetPosition(100);
        slidesL.setPower(1);
        slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        specimenClaw.setPosition(robot.SPECIMEN_CLAW_OPEN);

        robot.changeAccuracy(2,Math.toRadians(8));
        robot.changeSpeed(1,1);

        robot.goToPos(22,-23,Math.toRadians(0),Math.toRadians(-90));

        robot.goToPos(50,-20,Math.toRadians(0),Math.toRadians(0));

        robot.goToPos(50,-31,Math.toRadians(0),Math.toRadians(-90));

        robot.goToPos(11,-31,Math.toRadians(0),Math.toRadians(180));

    }

    public void secondPush(robotHardwarePinPoint robot) {

        robot.goToPos(50,-31,Math.toRadians(0),Math.toRadians(0));

        robot.goToPos(50,-43,Math.toRadians(0), Math.toRadians(-90));

        robot.goToPos(9,-42,Math.toRadians(0), Math.toRadians(180));

        robot.goToPos(11,-23,Math.toRadians(0), Math.toRadians(90));

    }

    public void thirdPush(robotHardwarePinPoint robot) {

        robot.changeAccuracy(1,Math.toRadians(1));
        robot.changeSpeed(0.75,0.75);

        x = 23;
        y = -29;
        finalAngle = Math.toRadians(-70);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(0));


            slidesR.setTargetPosition(100);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(100);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.mecanumDrive(0,0,0,0);

        extensionWrist.setPosition(robot.WRIST_PICKUP);
        turret.setPosition(robot.TURRET_LEFT);
        horizontalExtension.setTargetPosition(500);
        horizontalExtension.setPower(1);
        horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(500,robot.odometers);

        intake.setPower(-1);
        horizontalExtension.setTargetPosition(200);
        horizontalExtension.setPower(1);
        horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(250,robot.odometers);

        robot.goToPos(23,-29,Math.toRadians(-70),Math.toRadians(0));

        robot.wait(250,robot.odometers);

        robot.goToPos(23,-29,Math.toRadians(0),Math.toRadians(-70));

        robot.wait(250,robot.odometers);

        intake.setPower(1);

        robot.wait(250,robot.odometers);




    }

    public void firstCycle(robotHardwarePinPoint robot, int offset) {

        robot.changeAccuracy(4,Math.toRadians(5));

        slidesR.setTargetPosition(100);
        slidesR.setPower(1);
        slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesL.setTargetPosition(100);
        slidesL.setPower(1);
        slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        specimenClaw.setPosition(.5);

        specimenArm.setPosition(robot.SPECIMEN_ARM_PICKUP);
        specimenWrist.setPosition(robot.SPECIMEN_WRIST_PICKUP);

        robot.goToPos(5,-23,0,Math.toRadians(-130));

        robot.changeAccuracy(.5,Math.toRadians(1));
        robot.changeSpeed(1,1);

        deltaTime = currentTime.milliseconds() + 800;

        x = -1.5;
        y = -27;
        finalAngle = Math.toRadians(0);

        while((Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) && deltaTime > currentTime.milliseconds()) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(180));

        }

        specimenClaw.setPosition(1);
        robot.mecanumDrive(0,0,0,0);

        robot.wait(400,robot.odometers);

        slidesR.setTargetPosition(1250);
        slidesR.setPower(1);
        slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesL.setTargetPosition(1250);
        slidesL.setPower(1);
        slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        specimenArm.setPosition(robot.SPECIMEN_ARM_PLACE);
        specimenWrist.setPosition(robot.SPECIMEN_WRIST_PLACE);

        robot.wait(200,robot.odometers);


        robot.changeSpeed(1,1);
        robot.changeAccuracy(4,Math.toRadians(15));

        x = 25;
        y = 18 + offset;
        finalAngle = Math.toRadians(0);
        robot.goToPos(x,y,finalAngle,Math.toRadians(60));


        x = 34.5;
        y = 18;
        finalAngle = Math.toRadians(0);
        robot.changeAccuracy(1,Math.toRadians(1));

        deltaTime = currentTime.milliseconds() + 750;

        while((Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) && deltaTime > currentTime.milliseconds()){

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(0));

        }
        robot.mecanumDrive(0,0,0,0);

        robot.changeAccuracy(2,Math.toRadians(5));

        x = 26;
        y = 12;
        finalAngle = Math.toRadians(0);
        robot.goToPos(x,y,finalAngle,Math.toRadians(180));
        robot.mecanumDrive(0,0,0,0);

        robot.changeAccuracy(1,Math.toRadians(1));

    }

    public void park(robotHardwarePinPoint robot) {

        specimenClaw.setPosition(robot.SPECIMEN_CLAW_OPEN);
        specimenArm.setPosition(robot.SPECIMEN_ARM_PICKUP);

        robot.wait(200,robot.odometers);

        robot.changeAccuracy(5, Math.toRadians(10));

        slidesR.setTargetPosition(0);
        slidesR.setPower(1);
        slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesL.setTargetPosition(0);
        slidesL.setPower(1);
        slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

}