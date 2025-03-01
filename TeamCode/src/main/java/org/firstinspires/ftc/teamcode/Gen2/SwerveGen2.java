package org.firstinspires.ftc.teamcode.Gen2;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RI30HV2;

import java.util.Locale;


@TeleOp(name="TeleopGen2USETHISONE")
//@Disabled

public class SwerveGen2 extends LinearOpMode
{
    public DcMotor slidesR = null;
    public DcMotor slidesL = null;
    public DcMotor horizontalExtension = null;
    public DcMotor motorMTConverter = null;

    public boolean buttonY = true;
    public boolean buttonX = true;
    public boolean buttonA = true;
    public boolean buttonB = true;
    public boolean buttonLB = true;
    public boolean buttonLT = true;
    public boolean buttonRB = true;
    public boolean buttonRT = true;
    public boolean buttonDR = true;
    public boolean buttonDL = true;
    public boolean buttonDU = true;
    public boolean buttonDD = true;



    public boolean button2Y = true;
    public boolean button2X = true;
    public boolean button2A = true;
    public boolean button2B = true;
    public boolean button2LB = true;
    public boolean button2LT = true;
    public boolean button2RB = true;
    public boolean button2RT = true;
    public boolean button2DR = true;
    public boolean button2DL = true;
    public boolean button2DU = true;
    public boolean button2DD = true;

    public boolean upDown = false;
    public boolean out = false;
    public boolean in = true;
    public boolean prep = true;
    public boolean converter = false;
    public boolean B = false;
    public boolean Y = false;
    public boolean X = false;

    public int MTCoffset = 0;

    public boolean[] timerArray = new boolean[20];

    boolean upAlready = false;

    boolean doubleHang = false;

    public Servo extensionWrist = null;
    public CRServo intake = null;
    public Servo bucketArm = null;
    public Servo bucketWrist = null;
    public Servo turret = null;
    public Servo MTConverter = null;
    public Servo specimenArm = null;
    public Servo specimenWrist = null;
    public Servo specimenClaw = null;

    public double WristPosition = 0;
    public int SlidesPosition = 0;

    public double Position =.5;
    public double HEPosition =.1;
    public double APosition = .1;
    public double AWPosition = .35;
    public double SPEED = .75;

    double[] timeArray = new double[20];

    double time = 0;

    RI30HV2.AutoGrab autoGrab = RI30HV2.AutoGrab.START;

    @Override
    public void runOpMode() throws InterruptedException {

        robotHardwarePinPoint robot = new robotHardwarePinPoint(hardwareMap);

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
        motorMTConverter.setDirection(DcMotorSimple.Direction.REVERSE);

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        //intake.setPower(Position);...........

        while (!isStarted() && !isStopRequested()) {
            robot.odo.resetPosAndIMU();
            extensionWrist.setPosition(0);  //Tuned
            turret.setPosition(robot.TURRET_LEFT); //Tuned
            //intake.setPower(0);          //Tuned
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
            horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);  //Tuned   Smaller # is Out

            //motorMTConverter.setTargetPosition(0);
            //motorMTConverter.setPower(.2);
            //motorMTConverter.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }



        bucketWrist.setPosition(robot.BUCKET_WRIST_REST);
        bucketArm.setPosition(robot.BUCKET_ARM_REST);
        turret.setPosition(robot.TURRET_MIDDLE);
        extensionWrist.setPosition(.2);



        while (opModeIsActive()) {

            //robot.mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, SPEED); //normal people
            robot.mecanumDrive(gamepad1.right_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x, SPEED); //nolan

            robot.refresh(robot.odometers);


            intake(robot);

            bucket(robot);

            controller2(robot);

            slides(robot);

            //specimen(robot);

            robot.refresh(robot.odometers);

            telemetry.addData("turret", turret.getPosition());
            telemetry.addData("Arm", bucketArm.getPosition());
            telemetry.addData("ArmWrist", bucketWrist.getPosition());
            telemetry.addData("E_Wrist", Position);
            telemetry.addData("H_Extension", HEPosition);
            telemetry.addData("Slides", SlidesPosition);
            telemetry.addData("extension Wrist", extensionWrist.getPosition());
            telemetry.addData("Horizontal extension", horizontalExtension.getCurrentPosition());

            telemetry.addData("X",robot.GlobalX);
            telemetry.addData("Y",robot.GlobalY);

            //telemetry.addData("X",robot.odo.getEncoderX());
            //telemetry.addData("Y",robot.odo.getEncoderY());

            telemetry.addData("slideR",slidesR.getCurrentPosition());
            telemetry.addData("slideL",slidesL.getCurrentPosition());

            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            Pose2D pos = robot.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.update();

        }
    }

    public void intake(robotHardwarePinPoint robot){

        //intake

        if((gamepad1.right_bumper && buttonRB)/* || timerArray[1]**/ /*Add this to a if to be able to use timer "OR"*/){
            intake.setPower(-intake.getPower());
            buttonRB = false;

        }

        if(!gamepad1.right_bumper && !buttonRB){
            buttonRB = true;
        }


        if(upDown && gamepad1.right_trigger < .2 ){
            telemetry.addData("MainThing", true);
            if (out && in){
                telemetry.addData("Thing", true);
                in = false;
                timeArray[1] = robot.currentTime.milliseconds();//must have button press or will break
            }

            if (robot.boolTimer(timeArray[1] + 2000) ) {
                intake.setPower(0);
                out = false;
                in = true;
                extensionWrist.setPosition(.2);


            }
            else if(robot.boolTimer(timeArray[1] + 900) ){
                intake.setPower(1);

            }
            else{//first thing to happen
            }
        }

        //inc extension
        if(!converter){

            horizontalExtension.setTargetPosition((int) (gamepad1.right_trigger * 775) + 250);
            horizontalExtension.setPower(1);
            horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        //intake up and down
        if (gamepad1.left_trigger > .5 && buttonLT) {
            buttonLT = false;

            if (!upDown) {
                extensionWrist.setPosition(robot.WRIST_DROP);//upPos
                turret.setPosition(robot.TURRET_MIDDLE);
                //intake.setPower(1);
                upDown = true;
                out = true;
                SPEED = 1;

            }
            else if (upDown) {
                extensionWrist.setPosition(robot.WRIST_PICKUP);//downPos
                intake.setPower(-1);
                upDown = false;
                out = false;
                SPEED = .25;
            }
        }
        else if (gamepad1.left_trigger < .5 && !buttonLT) {
            buttonLT = true;
        }

    }

    public void bucket(robotHardwarePinPoint robot){

        if((gamepad1.left_bumper && buttonLB) || timerArray[0] /*Add this to a if to be able to use timer "OR"*/){
            if (gamepad1.left_bumper/*Boolean to start timer*/ && buttonLB) {
                timeArray[0] = robot.currentTime.milliseconds();//must have button press or will break
                timerArray[0] = true;
            }


            if (robot.currentTime.milliseconds() > timeArray[0] + 2100) {

                timerArray[0] = false;//If must be last timer, and must reset boolean when done

                SlidesPosition = 100;
                slidesR.setTargetPosition(SlidesPosition);
                slidesL.setTargetPosition(SlidesPosition);
                slidesR.setPower(1);
                slidesL.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            else if (robot.boolTimer(timeArray[0] + 1600)) {
                bucketArm.setPosition(robot.BUCKET_ARM_REST);//going to rest position/down
                bucketWrist.setPosition(robot.BUCKET_WRIST_REST);//rest
            }
            else{//first thing to happen

                bucketArm.setPosition(robot.BUCKET_ARM_DROP);//dropping
                bucketWrist.setPosition(robot.BUCKET_WRIST_DROP);//drop

            }
            buttonLB = false;
            telemetry.addData("booo5100", timeArray);
            telemetry.addData("currentTime", robot.currentTime.milliseconds());
        }

        if(!gamepad1.left_bumper && !buttonLB){
            buttonLB = true;
        }

    }
    public void controller2(robotHardwarePinPoint robot){

        if(gamepad2.dpad_right && button2DR){
            turret.setPosition(robot.TURRET_RIGHT);
            button2DR = false;
        }

        if(!gamepad2.dpad_right && !button2DR){
            button2DR = true;

        }

        if(gamepad2.dpad_left && button2DL){
            turret.setPosition(robot.TURRET_LEFT);
            button2DL = false;

        }

        if(!gamepad2.dpad_left && !button2DL){
            button2DL = true;

        }


        if((gamepad2.dpad_up && button2DU)){
            turret.setPosition(robot.TURRET_MIDDLE);
            button2DU = false;

        }

        if(!gamepad2.dpad_up && !button2DU){
            button2DU = true;

        }



        if((gamepad2.y && button2Y) && !doubleHang){//going up
            if(converter){
                motorMTConverter.setTargetPosition(7020);
                motorMTConverter.setPower(1);
                motorMTConverter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            slidesR.setTargetPosition(1900);
            slidesL.setTargetPosition(1900);
            slidesR.setPower(1);
            slidesL.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            upAlready = false;
            button2Y = false;

        }
        else if(gamepad2.y && button2Y){
            if(converter){
                motorMTConverter.setTargetPosition(1040);
                motorMTConverter.setPower(1);
                motorMTConverter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            slidesR.setTargetPosition(280);
            slidesL.setTargetPosition(280);
            slidesR.setPower(1);
            slidesL.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            doubleHang = false;
            button2Y = false;
        }




        if(!gamepad2.y && !button2Y){
            button2Y = true;

        }



        if(gamepad2.b || timerArray[15]){//prep
            if (!timerArray[15]){
                timeArray[15] = robot.currentTime.milliseconds();//must have button press or will break
                timerArray[15] = true;
            }

            if(robot.boolTimer(timeArray[15] + 2000)){
                MTConverter.setPosition(.85);
            }
            else if (robot.boolTimer(timeArray[15] + 1500) ) {


                slidesR.setTargetPosition(0);
                slidesL.setTargetPosition(0);
                slidesR.setPower(1);
                slidesL.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            else if(robot.boolTimer(timeArray[15] + 500) ){
                bucketWrist.setPosition(1);
                bucketArm.setPosition(.35);
                turret.setPosition(.2);
                extensionWrist.setPosition(.4);
                intake.setPower(0);

                specimenArm.setPosition(.1);
                specimenClaw.setPosition(1);
                specimenWrist.setPosition(.65);

                horizontalExtension.setTargetPosition(0);
                horizontalExtension.setPower(1);
                horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            else{//first thing to happen

                horizontalExtension.setTargetPosition(300);
                horizontalExtension.setPower(1);
                horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }




            MTCoffset = slidesR.getCurrentPosition();
            converter = true;
        }

        if(gamepad2.dpad_down){

            bucketWrist.setPosition(.6);
            bucketArm.setPosition(.6);
            specimenArm.setPosition(robot.SPECIMEN_ARM_PLACE);

        }

        if(gamepad2.a && button2A){//going down
            if(converter && !upAlready){

                motorMTConverter.setTargetPosition(0);
                motorMTConverter.setPower(1);
                motorMTConverter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                upAlready = true;
                doubleHang = true;
            }
            else {
                motorMTConverter.setTargetPosition(motorMTConverter.getCurrentPosition() - 500);
                motorMTConverter.setPower(1);
                motorMTConverter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            slidesR.setTargetPosition(0);
            slidesL.setTargetPosition(0);
            slidesR.setPower(1);
            slidesL.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            button2A = false;

        }

        if(!gamepad2.a && !button2A){

            button2A = true;
        }


        //manual reset
        if(gamepad2.left_bumper && button2LB){//slides fine adjust

            SlidesPosition -= 100;
            slidesR.setTargetPosition(SlidesPosition);
            slidesL.setTargetPosition(SlidesPosition);
            slidesR.setPower(1);
            slidesL.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            button2LB = false;

        }

        if(!gamepad2.left_bumper && !button2LB){

            button2LB = true;
        }

        if(gamepad2.right_bumper && button2RB){

            SlidesPosition += 100;
            slidesR.setTargetPosition(SlidesPosition);
            slidesL.setTargetPosition(SlidesPosition);
            slidesR.setPower(1);
            slidesL.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            button2RB = false;

        }

        if(!gamepad2.right_bumper && !button2RB){

            button2RB = true;
        }

        if(gamepad2.x && button2X){//resetting slide's encoder

            slidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            button2X = false;

        }

        if(!gamepad2.x && !button2X){

            button2X = true;
        }






        // Pick up and place specimen
        if((gamepad2.right_trigger >.5 && button2RT) || timerArray[12] /*Add this to a if to be able to use timer "OR"*/){
            if (gamepad2.right_trigger >.5/*Boolean to start timer*/ && button2RT) {
                timeArray[12] = robot.currentTime.milliseconds();//must have button press or will break
                timerArray[12] = true;
            }


            if (robot.currentTime.milliseconds() > timeArray[12] + 1000) {

                timerArray[12] = false;//If must be last timer, and must reset boolean when done

                specimenArm.setPosition(robot.SPECIMEN_ARM_PLACE);
                specimenWrist.setPosition(robot.SPECIMEN_WRIST_PLACE);


            }
            else if (robot.boolTimer(timeArray[12] + 300)) {
                SlidesPosition = 1250; //Position Not Correct
                slidesR.setTargetPosition(SlidesPosition);
                slidesL.setTargetPosition(SlidesPosition);
                slidesR.setPower(1);
                slidesL.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else{//first thing to happen
                specimenClaw.setPosition(1);
            }

            button2RT = false;

        }

        if(gamepad2.right_trigger <.5 && !button2RT){
            button2RT = true;
        }
        //Reset Specimen
        if(gamepad2.left_trigger > .5 && button2LT){

            specimenArm.setPosition(robot.SPECIMEN_ARM_PICKUP);
            specimenClaw.setPosition(.5);
            specimenWrist.setPosition(robot.SPECIMEN_WRIST_PICKUP);

            SlidesPosition = (int)robot.SLIDE_INIT;
            slidesR.setTargetPosition(SlidesPosition);
            slidesL.setTargetPosition(SlidesPosition);
            slidesR.setPower(1);
            slidesL.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            button2LT = false;
        }

        if(gamepad2.left_trigger <.5 && !button2LT){
            button2LT = true;
        }


    }

    public void slides(robotHardwarePinPoint robot){

        if(gamepad1.dpad_up && buttonDU && SlidesPosition < robot.SLIDE_TOP){
            SlidesPosition = (int) robot.SLIDE_TOP;
            slidesR.setTargetPosition(SlidesPosition);
            slidesL.setTargetPosition(SlidesPosition);
            slidesR.setPower(1);
            slidesL.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            specimenWrist.setPosition(robot.SPECIMEN_WRIST_SLIDES);

            intake.setPower(0);
            extensionWrist.setPosition(.2);

            buttonDU = false;
            SPEED = .5;
        }

        if(gamepad1.dpad_left && buttonDL && SlidesPosition < robot.SLIDE_MID){
            SlidesPosition = (int) robot.SLIDE_MID;
            slidesR.setTargetPosition(SlidesPosition);
            slidesL.setTargetPosition(SlidesPosition);
            slidesR.setPower(1);
            slidesL.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            specimenWrist.setPosition(robot.SPECIMEN_WRIST_SLIDES);
            buttonDL = false;
            SPEED = .5;
        }

        if(gamepad1.dpad_down && buttonDD && SlidesPosition > 100){
            SlidesPosition = (int) robot.SLIDE_INIT;
            slidesR.setTargetPosition(SlidesPosition);
            slidesL.setTargetPosition(SlidesPosition);
            slidesR.setPower(1);
            slidesL.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            specimenWrist.setPosition(robot.SPECIMEN_WRIST_SLIDES);
            buttonDD = false;
            SPEED = 1;
        }


        if(!gamepad1.dpad_up && !buttonDU){
            buttonDU = true;
        }

        if(!gamepad1.dpad_left && !buttonDL){
            buttonDL = true;
        }

        if(!gamepad1.dpad_down && !buttonDD){
            buttonDD = true;
        }

    }
    public void specimen(robotHardwarePinPoint robot){

        //Hanging the specimen
        if(gamepad1.a && buttonA){

            specimenArm.setPosition(robot.SPECIMEN_ARM_PLACE);
            specimenWrist.setPosition(robot.SPECIMEN_WRIST_PLACE);

            buttonA = false;

        }

        if(!gamepad1.a && !buttonA){
            buttonA = true;

        }


    }
}
