package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="Gen1Teleop")
@Disabled

public class RI30HV2 extends LinearOpMode {
    public boolean buttonB = true;
    public boolean buttonA = true;
    public boolean buttonC = true;
    public boolean openClose = true;
    public boolean buttonD = true;
    public boolean buttonE = true;

    public Servo armWrist = null;
    public CRServo intake = null;
    public Servo slideFingerR = null;
    public Servo slideFingerL = null;
    public Servo dumper = null;
    public Servo specimenGrab = null;

    public DcMotor arm = null;
    public DcMotor slides = null;

    public int slideLoops = -1;
    public int slideEncoder = 0;
    public int offset = 0;
    public double speed = 1;
    public double armPower = 0;

    public int maxSlides = 4500;
    public int downPos = 660;
    public int armEncoder = 0;

    public boolean down = false;
    public boolean open = false;
    public boolean outIn = false;
    public boolean manualReset = false;

    public boolean x = false;

    double time = 0;
    public boolean timerInit2 = false;
    public boolean timerInit3 = false;
    public boolean timerInit4 = false;
    public boolean timerInit5 = false;


    public enum AutoGrab {
        START,
        GRAB_SAMPLE,
        DROP_SAMPLE,
        TOP_SLIDE_POS,
        MIDDLE_SLIDE_POS,
        BOTTOM_SLIDE_POS,
        ARM_TOP_POS,
        ARM_MIDDLE_POS,
        ARM_BOTTOM_POS,
        ARM_SPECIMEN_POS,
        MANUAL
    }

    AutoGrab autoGrab = AutoGrab.START;



    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        robot.odo.resetPosAndIMU();

        //drive motors
        arm = hardwareMap.dcMotor.get("frontArmMotor");
        slides = hardwareMap.dcMotor.get("slides");


        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slides.setDirection(DcMotorSimple.Direction.REVERSE);

        armWrist = hardwareMap.servo.get("intakeWrist");
        intake = hardwareMap.crservo.get("intake");
        //slideFingerL = hardwareMap.servo.get("");
        //slideFingerR = hardwareMap.servo.get("");
        dumper = hardwareMap.servo.get("bucket");
        specimenGrab = hardwareMap.servo.get("specimenGrab");

        //arm.setDirection(DcMotor.Direction.REVERSE);


        while (!isStarted() && !isStopRequested()) {


            //smaller numbers go up higher
            dumper.setPosition(0.35);
            armWrist.setPosition(.3);
            specimenGrab.setPosition(robot.SPECIMEN_OPEN);//closed is .1, open is .45

            intake.setPower(0);

            //arm.setTargetPosition(0);
            //arm.setPower(0.175);
            //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        arm.setTargetPosition(200);
        arm.setPower(0.175);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (opModeIsActive()) {

            //robot.mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, speed); //normal people
            robot.mecanumDrive(gamepad1.right_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x, speed); //nolan


            //Bucket
            if ((gamepad1.a && buttonA) || timerInit2) {
                // down position
                if (gamepad1.a) {
                    time = robotHardware.currentTime.milliseconds() + 250;
                    timerInit2 = true;
                }


                if (robot.boolTimer(time + 1000)) {
                    //slideEncoder = 0;
                    timerInit2 = false;
                }
                else if (robot.boolTimer(time + 500)) {
                    dumper.setPosition(.70);//higher number brings the dumper down
                } else {
                    dumper.setPosition(.55);
                }

                buttonA = false;

            }
            if (!gamepad2.a && !buttonA) {
                buttonA = true;
            }


            if (gamepad1.b) {
                dumper.setPosition(.35);
            }
            //Front Claw
            //if (gamepad1.x){
            //    armFingerL.setPosition(0.7);
            //
            //}
            //if (gamepad1.y){
            //    armFingerL.setPosition(0.6);
            //
            //}





            //slide fine adjust
           /*if (gamepad2.y && buttonA) {
                slideEncoder += 200;
                buttonA = false;
                intake.setPower(0);
            }


           else if (gamepad2.x && buttonB) {

                slideEncoder -= 200;
                buttonB = false;
                intake.setPower(0);

           }
           if (!gamepad2.a && !buttonA) {
               buttonA = true;
           }
           else if (!gamepad2.y && !buttonB) {
               buttonB = true;
           }

            */


           if (gamepad1.left_stick_button) {

               downPos = arm.getCurrentPosition() - 5;
               arm.setTargetPosition(robot.downPos);
               arm.setPower(.4);
               arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           }

            if (gamepad1.right_stick_button) {

                downPos = arm.getCurrentPosition() + 5;
                arm.setTargetPosition(downPos);
                arm.setPower(.4);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }


            //One button press
            if (gamepad1.right_trigger > .5 && buttonC) {
                buttonC = false;


                if (openClose) {
                    autoGrab = AutoGrab.DROP_SAMPLE;

                }
                else if (!openClose) { //this closes
                    autoGrab = AutoGrab.GRAB_SAMPLE;
                }

                open = false;
            }
            else if (gamepad1.right_trigger < .5 && !buttonC) {
                buttonC = true;
            }


            //Arm

            if (gamepad1.dpad_up || robotHardware.timerInitted) {//back position
                if (gamepad1.dpad_up) {
                    time = robot.timerInit(700);
                    robotHardware.timerInitted=true;
                }

                dumper.setPosition(.35);

                autoGrab = AutoGrab.ARM_TOP_POS;

                if (robot.boolTimer(time + 1000)) {
                    autoGrab = AutoGrab.ARM_MIDDLE_POS;
                    intake.setPower(0);
                    robotHardware.timerInitted = false;
                } else if (robot.boolTimer(time)) {
                    autoGrab = AutoGrab.DROP_SAMPLE;

                }

            }


            if (gamepad1.dpad_right) {//collect prep
                autoGrab = AutoGrab.ARM_MIDDLE_POS;
            }
            if (gamepad1.dpad_down) {//bottom position for arm
                autoGrab = AutoGrab.ARM_BOTTOM_POS;
                intake.setPower(.75);
            }

            if (gamepad1.left_bumper && buttonD) {// 8700 is max
                autoGrab = AutoGrab.TOP_SLIDE_POS;
                //slideEncoder = maxSlides;


                buttonD = false;
            }
            if (!gamepad1.left_bumper && !buttonD) {
                buttonD = true;
            }

            if (gamepad1.left_trigger > .5){

                autoGrab = AutoGrab.MIDDLE_SLIDE_POS;

            }
            if (gamepad1.right_bumper) {

                autoGrab = AutoGrab.BOTTOM_SLIDE_POS;
                //slideEncoder = 0;
            }


            if ((gamepad1.dpad_left && buttonE) || timerInit4){
                if (gamepad1.dpad_left) {
                    time = robot.timerInit(250);
                    timerInit4 = true;
                    intake.setPower(0);
                }

                autoGrab = AutoGrab.ARM_SPECIMEN_POS;

                if (outIn){
                    if (slideEncoder == 2000 || slideEncoder == 1250){
                        slideEncoder = 1250;

                        if(robot.boolTimer(time + 350)){
                            specimenGrab.setPosition(robot.SPECIMEN_OPEN);//opening
                            outIn = false;
                            timerInit4 = false;
                        }
                    }
                    else {
                        specimenGrab.setPosition(robot.SPECIMEN_OPEN);//opening
                        outIn = false;
                        timerInit4 = false;
                    }

                }

                else if (!outIn){
                    specimenGrab.setPosition(robot.SPECIMEN_CLOSE);

                    if(robot.boolTimer(time)) {
                        slideEncoder = 500;
                        outIn = true;
                        timerInit4 = false;
                    }
                }

                buttonE = false;
            }
            if(!gamepad1.dpad_left && !buttonE){
                buttonE = true;
            }



            //if (gamepad1.right_bumper){
            //    slideEncoder = 0;
            //}


            switch (autoGrab) {
                case GRAB_SAMPLE:


                    openClose = true;
                    intake.setPower(.75);

                    break;
                case DROP_SAMPLE:

                    openClose = false;
                    intake.setPower(-1);

                    break;
                case TOP_SLIDE_POS:
                    slideEncoder = maxSlides;
                    slideLoops ++;

                    break;
                case MIDDLE_SLIDE_POS:
                    slideEncoder = 2000;

                    break;
                case BOTTOM_SLIDE_POS:
                    slideEncoder = 0;

                    break;
                case ARM_TOP_POS:

                    armWrist.setPosition(robot.WRIST_TOP);
                    armEncoder = robot.ARM_TOP;
                    armPower = .4;


                    break;
                case ARM_MIDDLE_POS:
                    armWrist.setPosition(robot.WRIST_MID);
                    armEncoder = robot.ARM_MID;
                    armPower = .25;
                    //intake.setPower(0);


                    break;
                case ARM_BOTTOM_POS:

                    armWrist.setPosition(robot.WRIST_LOW);
                    armEncoder = robot.ARM_LOW;
                    armPower = .4;

                    break;
                case ARM_SPECIMEN_POS:

                    armEncoder = 200;
                    armPower = .4;

                    break;

            }


            if (slides.getCurrentPosition() > 6000 || arm.getCurrentPosition() > 550) {
                speed = .5;
            } else {
                speed = 1;
            }



            //Slides stops
            if (slideEncoder < 0) {
                slideEncoder = 0;
            }
            if (slideEncoder > maxSlides) {
                //slideEncoder = maxSlides;
            }

            //More slides stuff
            //4500 is the max for the slides
            /*
            if (slides.getCurrentPosition() < slideEncoder) {
                slides.setTargetPosition(slideEncoder);
                slides.setPower(1);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else {
                slides.setTargetPosition(slideEncoder);
                slides.setPower(.5);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }time = robotHardware.currentTime.milliseconds();

             */


            slides.setTargetPosition(slideEncoder + offset);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(gamepad2.y){

                manualReset = true;
                arm.setPower(.4);
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if(gamepad2.b){
                if(gamepad2.dpad_right){
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    manualReset = false;
                }
                arm.setPower(-.4);
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }

            if(armEncoder > 600 && arm.getCurrentPosition() > 600 && !manualReset){
                arm.setTargetPosition(armEncoder);
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if(!manualReset){
                arm.setTargetPosition(armEncoder);
                arm.setPower(armPower);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            // Slides and Arm Manual Reset
            if((gamepad2.a && gamepad2.x) || timerInit5){
                slideEncoder = -4200;
                autoGrab = AutoGrab.ARM_SPECIMEN_POS;
                slides.setTargetPosition(slideEncoder);
                slides.setPower(1);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (gamepad2.a) {
                    time = robotHardware.currentTime.milliseconds() + 3000;
                    timerInit5 = true;
                }
                if (robot.boolTimer(time)) {
                    //slideEncoder = 0;
                    timerInit5 = false;
                    slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }


            }





            //robot.odo.update();
            robot.refresh(robot.odometers);


            telemetry.addData("slide encoder", slideEncoder + offset);
            telemetry.addData("slide pos", slides.getCurrentPosition());

            telemetry.addData("", null);

            telemetry.addData("X", robot.GlobalX);
            telemetry.addData("Y", robot.GlobalY);
            telemetry.addData("Heading", Math.toDegrees(robot.angleWrapRad(robot.GlobalHeading)));


            telemetry.addData("", null);

            telemetry.addData("currentRightPos", robot.odometers[0].getCurrentPosition());
            telemetry.addData("currentLeftPos", robot.odometers[1].getCurrentPosition());
            telemetry.addData("currentPerpendicularPos", robot.odometers[2].getCurrentPosition());

            telemetry.addData("armPos", arm.getCurrentPosition());

            telemetry.addData("getPosX", robot.odo.getPosX());
            telemetry.addData("getEncoderX", robot.odo.getEncoderX());
            telemetry.update();


        }

    }






}
