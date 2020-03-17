
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;


@TeleOp(name="Drive Me", group="Pushbot")

public class Teleop_test extends LinearOpMode {

    /* Declare OpMode members. */
    FairBotHardware robot           = new FairBotHardware();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
double x1, y1, y2;
double lift;
double winch;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) 
        {
            if((gamepad2.left_stick_y > .2 || gamepad1.right_bumper))
            {
                if(gamepad1.right_bumper)
                {
                    lift = 1;
                }
                else
                {
         
                        lift = gamepad2.left_stick_y;
                }
                }
                
            
            else if((gamepad2.left_stick_y < -.2 || gamepad1.left_bumper))
            {
               if(gamepad1.left_bumper)
               {
                   lift = -1;
               }
               else
               {
         
                       lift = gamepad2.left_stick_y;

                   }
                    }  
            else{
                lift=0;}
            
            
            robot.lift.setPower(-lift);
            robot.L2.setPower(-lift);


    if(Math.abs(gamepad1.left_stick_y)>.2){
x1 = -gamepad1.left_stick_y;}
else{ 
    x1 = 0;}
    if(Math.abs(gamepad1.left_stick_x)>.2){
y1 = gamepad1.left_stick_x;}
else{ 
    y1 = 0;}
    if(Math.abs(gamepad1.right_stick_x)>.2){
        y2 = gamepad1.right_stick_x;}
        else{
        y2 = 0;}
        
double fl= x1+y1+y2;
double rl= x1-y1+y2;
double fr= x1-y1-y2;
double rr= x1+y1-y2;
double ML=Math.max(Math.abs(fl),Math.abs(rl));
double MR=Math.max(Math.abs(fr),Math.abs(rr));
double max = Math.max(ML,MR);
if(max>1){
fl/=max;
rl/=max;
fr/=max;
rr/=max;
}

if(gamepad1.left_trigger>.4){
robot.flm.setPower(fl/2);
robot.rlm.setPower(rl/2);
robot.frm.setPower(fr/2);
robot.rrm.setPower(rr/2);
    
}
else if(gamepad1.right_trigger>.2){
    double mod=1-gamepad1.right_trigger;
robot.flm.setPower(fl*mod);
robot.rlm.setPower(rl*mod);
robot.frm.setPower(fr*mod);
robot.rrm.setPower(rr*mod);   
}
else{
 robot.flm.setPower(fl);
robot.rlm.setPower(rl);
robot.frm.setPower(fr);
robot.rrm.setPower(rr);   
    
}

if(gamepad2.a && robot.TouchHigh.getState()){
    robot.claw.setPosition(.12);}
    else if(gamepad2.b && robot.TouchHigh.getState()){
    robot.claw.setPosition(.8);}

if(gamepad1.a){
    robot.base.setPosition(1);
    robot.base2.setPosition(0);
}
    else if (gamepad1.b){
        robot.base.setPosition(0);
        robot.base2.setPosition(1);
    }
    

if(Math.abs(gamepad2.right_stick_y)>.2){
    winch = -gamepad2.right_stick_y;}
else{
    winch= 0;}
    
    robot.winch.setPower(-winch);
    telemetry.addData("max", max);
telemetry.addData("fl", fl);
telemetry.addData("rl", rl);
telemetry.addData("fr", fr);
telemetry.addData("rr", rr);
    telemetry.update();
        }
        
    }




        }
