package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import java.util.List;



import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
@Disabled
@Autonomous(name = "BLUE block", group = "Concept")

public class Mecanum_testing_test extends LinearOpMode {
         BNO055IMU imu;
          // NormalizedColorSensor colorSensor;
             Orientation lastAngles = new Orientation();

    Orientation angles;
            double globalAngle;
           FairBotHardware robot           = new FairBotHardware();//links code to main hardware map
    private ElapsedTime runtime = new ElapsedTime();


    // values is a reference to the hsvValues array.
    //float[] hsvValues = new float[3];
    //final float values[] = hsvValues;

    // bPrevState and bCurrState keep track of the previous and current state of the button
    boolean bPrevState = false;
    boolean bCurrState = false;
double power=0;
double c_angle;
double Block1, Block2;
    // Get a reference to our sensor object.
   


@Override
    public void runOpMode() throws InterruptedException{
          robot.init(hardwareMap);
                       //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");




robot.frm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.flm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rrm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rlm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        robot.frm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rrm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rlm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
        int Distance;
         //Gyro
          BNO055IMU.Parameters Gparameters = new BNO055IMU.Parameters();
          
                 
        
        //GYRO
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(Gparameters);
        
        //GYRO
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {

        telemetry.addData("Mode", "init");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        }
        Orientation angles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        
        
        
        
        
        
        
        
        
        
        
        int Block = 1;
        int seen = 0;
        
                       
                       float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        
        // wait for the start button to be pressed.
        waitForStart();

        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                    (int) (robot.sensorColor.green() * SCALE_FACTOR),
                    (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", robot.sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", robot.sensorColor.alpha());
            telemetry.addData("Red  ", robot.sensorColor.red());
            telemetry.addData("Green", robot.sensorColor.green());
            telemetry.addData("Blue ", robot.sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);

                       
                       
                       
                       
                       
                       
                       
                       
      extend(300);                 
lift(2000);
      extend(1800);
      robot.claw.setPosition(.8);
    forward(22);
     sleep(500);
    turn(95);
     sleep(500);
    forward(-9);
    
    
   // while(opModeIsActive() && seen==0){
  // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                    (int) (robot.sensorColor.green() * SCALE_FACTOR),
                    (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                    
                    telemetry.addData("Alpha", robot.sensorColor.alpha());
            telemetry.addData("Red  ", robot.sensorColor.red());
            telemetry.addData("Green", robot.sensorColor.green());
            telemetry.addData("Blue ", robot.sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("sat", hsvValues[1]);
            telemetry.addData("Val", hsvValues[2]);
                    telemetry.update();
      
      
      Block1 = hsvValues[2];
      forward(-8);
      Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                    (int) (robot.sensorColor.green() * SCALE_FACTOR),
                    (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                    
                    telemetry.addData("Alpha", robot.sensorColor.alpha());
            telemetry.addData("Red  ", robot.sensorColor.red());
            telemetry.addData("Green", robot.sensorColor.green());
            telemetry.addData("Blue ", robot.sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("sat", hsvValues[1]);
            telemetry.addData("Val", hsvValues[2]);
                    telemetry.update();
      
      Block2 = hsvValues[2];
      
      if( Block1 + 1 > Block2 && Block2 + 1 > Block1){
          forward(-8);//drive to block 3
          turn(-83);
          
      lift(-1500);

          robot.claw.setPosition(.8);
          forward(6);
          sleep(100);
         // lift(2000);
          Block = 3;
      }
      else if(Block1 + 2 < Block2){
          forward(8);
          turn(-83);

      lift(-1500);
          robot.claw.setPosition(.8);
          forward(6);
          sleep(100);
         // lift(2000);
          Block = 1;
      }
      else{//grab block 2
          turn(-83);    
      
      lift(-1500);
          robot.claw.setPosition(.8);
          forward(6);
          sleep(100);
        //  lift(2000);
          Block = 2;
      }
      robot.claw.setPosition(.12);
      forward(-7);
      turn(95);
      forward((Block*8)+31);
      
      
      
      
      
      
      
      
      
       /* if(hsvValues[2]<19){
           seen=1;
            turn(85);
            extend(-2000);
            robot.claw.setPosition(.8);
            forward(6);
            
            sleep(100);
            lift(4000);

        
        }
        else if(Block==5){
                       seen=1;
            turn(85);
            extend(-2000);
            robot.claw.setPosition(.8);
            forward(6);
            
            sleep(100);
            lift(4000);

        }
        else{
            
            Block++;
            forward(7);} */
        
        
   // }
              telemetry.addData("Block1", Block1);
            telemetry.addData("Block2", Block2);
            telemetry.update();
   
   
   /* forward(-4);
    robot.claw.setPosition(.1);
    sleep(600);
    lift(-1500);
    turn(90);
    forward((Block*8)+40);
    lift(-2000);
    forward(10);
    robot.claw.setPosition(.8);
    extend(-1000);
    turn(170);
    extend(2000);
    lift(2000);
    forward(30);*/
      
      

      
      sleep(10000000);
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
    }
       public void left(int Distance){
      
                   int new_d=Distance *53;
    robot.frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    //target position
    robot.frm.setTargetPosition(-new_d);
    robot.flm.setTargetPosition(new_d);
    robot.rrm.setTargetPosition(new_d);
    robot.rlm.setTargetPosition(-new_d);
    
    //set to Run_TO_POSITION OpMode
    robot.frm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.flm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rrm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rlm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    // Set drive power
    robot.frm.setPower(-.25);
    robot.flm.setPower(-.25);
    robot.rrm.setPower(.25);
    robot.rlm.setPower(.25);
    
    while(opModeIsActive()&&robot.frm.isBusy() && robot.flm.isBusy() && robot.rrm.isBusy() && robot.rlm.isBusy()){

}
    robot.frm.setPower(0);
    robot.flm.setPower(0);
    robot.rrm.setPower(0);
    robot.rlm.setPower(0);


           
                      
    }
public void right(int Distance){
      
                   int new_d=Distance *53;
    robot.frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    //target position
    robot.frm.setTargetPosition(new_d);
    robot.flm.setTargetPosition(-new_d);
    robot.rrm.setTargetPosition(-new_d);
    robot.rlm.setTargetPosition(new_d);
    
    //set to Run_TO_POSITION OpMode
    robot.frm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.flm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rrm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rlm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    // Set drive power
    robot.frm.setPower(-.25);
    robot.flm.setPower(-.25);
    robot.rrm.setPower(.25);
    robot.rlm.setPower(.25);
    
    while(opModeIsActive()&&robot.frm.isBusy() && robot.flm.isBusy() && robot.rrm.isBusy() && robot.rlm.isBusy()){
    
}
    robot.frm.setPower(0);
    robot.flm.setPower(0);
    robot.rrm.setPower(0);
    robot.rlm.setPower(0);


           
                      
    }
public void lift(int Distance){
    robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.L2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
   // robot.lift.setTargetPosition(Distance);
    //robot.L2.setTargetPosition(Distance);
    
    robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.L2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
    
    
    if(Distance < 0){
    robot.lift.setPower(-1);
    robot.L2.setPower(-1);
    
    while(opModeIsActive()&&(robot.lift.getCurrentPosition()>Distance)&&(robot.L2.getCurrentPosition()>Distance)){}
    }
     else if(Distance > 0){
    robot.lift.setPower(1);
    robot.L2.setPower(1);
    
    while(opModeIsActive()&&(robot.lift.getCurrentPosition()<Distance)&&(robot.L2.getCurrentPosition()<Distance)){}
    }
    
    
   
    robot.lift.setPower(0);
    robot.L2.setPower(0);
}
public void extend(int Distance){
     robot.winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    robot.winch.setTargetPosition(Distance);
    
    robot.winch.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    if(Distance<0){
    robot.winch.setPower(-1);
    
    while(opModeIsActive()&&(robot.winch.getCurrentPosition()>Distance)){}
    }
        if(Distance>0){
    robot.winch.setPower(1);
    
    while(opModeIsActive()&&(robot.winch.getCurrentPosition()<Distance)){}
    }
    robot.winch.setPower(0);
    
}
public void forward(int Distance){
      double rpower, lpower, ln, rn;

                   Distance =Distance *40;
    robot.frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    while(opModeIsActive()&&(Math.abs(robot.frm.getCurrentPosition())>0) && (Math.abs(robot.rrm.getCurrentPosition())>0) && (Math.abs(robot.flm.getCurrentPosition())>0) && (Math.abs(robot.rlm.getCurrentPosition())>0)){}
      resetAngle();

    //set to Run_TO_POSITION OpMode
    robot.frm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.flm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rrm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rlm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
  runtime.reset();  
if(Distance >0){
 while(opModeIsActive()&&(robot.frm.getCurrentPosition()<Distance) && (robot.flm.getCurrentPosition()<Distance) && (robot.rrm.getCurrentPosition()<Distance) && (robot.rlm.getCurrentPosition()<Distance)){
if( runtime.seconds() < 2){
    rpower=.6 * (runtime.seconds()/2);
    lpower=.6 * (runtime.seconds()/2);
}
else{
    rpower=.6;
    lpower=.6;
}
    c_angle = getAngle();
        telemetry.addData("Right", robot.frm.getCurrentPosition());
        telemetry.addData("Left", robot.flm.getCurrentPosition());
        telemetry.addData("Rear Right", robot.rrm.getCurrentPosition());
        telemetry.addData("Real Left", robot.rlm.getCurrentPosition());
        telemetry.addData("Gyro", c_angle);
    telemetry.update();
        double down = Distance-robot.frm.getCurrentPosition();
    if(down <101&&down>10){
        rpower*=(down/100);
        lpower*=(down/100);
    }
    else if(down<10){
        rpower=.15;
        lpower=.15;
        
    }
    
   /* if(c_angle < -0.1){
        rn = rpower+(-c_angle/10);
        ln=lpower;
    }
    else if(c_angle > 0.1){
      ln = lpower+(c_angle/10);
      rn=rpower;
    }
    else{
        rn=rpower;
        ln=lpower;
    }*/

          rn=rpower;
        ln=lpower;  
    robot.frm.setPower(rn);
    robot.flm.setPower(ln);
    robot.rrm.setPower(rn);
    robot.rlm.setPower(ln);
 }
}
else if(Distance<0){
   while(opModeIsActive()&&(robot.frm.getCurrentPosition()>Distance) && (robot.flm.getCurrentPosition()>Distance) && (robot.rrm.getCurrentPosition()>Distance) && (robot.rlm.getCurrentPosition()>Distance)){
if( runtime.seconds() < 1){
    rpower=-.55 * runtime.seconds();
    lpower=-.55 * runtime.seconds();
}
else{
    rpower=-.55;
    lpower=-.55;
}
    // Set drive power

    c_angle = getAngle();
     /*   telemetry.addData("Right", robot.frm.getCurrentPosition());
        telemetry.addData("Left", robot.flm.getCurrentPosition());
        telemetry.addData("Rear Right", robot.rrm.getCurrentPosition());
        telemetry.addData("Real Left", robot.rlm.getCurrentPosition());
        telemetry.addData("Gyro", c_angle);*/
    telemetry.update();
    
        double down = Distance-robot.frm.getCurrentPosition();
    if(down <81&&down>10){
        rpower*=(down/80);
        lpower*=(down/80);
    }
    else{
        rpower=-.15;
        lpower=-.15;
        
    }
    
  /*  if(c_angle < -0.02){
        rn = rpower+(-c_angle/10);
        ln=lpower;
    }
    else if(c_angle > 0.02){
      ln = lpower+(c_angle/10);
      rn=rpower;
    }
    else{
        rn=rpower;
        ln=lpower;
    }*/
    
    
            rn=rpower;
        ln=lpower;
   robot.frm.setPower(rn);
    robot.flm.setPower(ln);
    robot.rrm.setPower(rn);
    robot.rlm.setPower(ln);
 }
   
}
    
    
    robot.frm.setPower(0);
    robot.flm.setPower(0);
    robot.rrm.setPower(0);
    robot.rlm.setPower(0);
   /* if((new_d-robot.rlm.getCurrentPosition())>10 || (new_d-robot.flm.getCurrentPosition())>10 ){
        robot.rlm.setPower(.5);
        robot.flm.setPower(.5);
    }
    else if((new_d-robot.rrm.getCurrentPosition())>10 || (new_d-robot.frm.getCurrentPosition())>10 ){
        robot.rrm.setPower(.5);
        robot.frm.setPower(.5);
    }

    robot.frm.setPower(0);
    robot.flm.setPower(0);
    robot.rrm.setPower(0);
    robot.rlm.setPower(0);*/
           
                      
    }
public void turn(int Distance){
     resetAngle();
    double rpower, lpower, ln, rn;
    double power = .6;
      runtime.reset();
                   Distance=Distance *9;
    robot.frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
   
    //set to Run_TO_POSITION OpMode
    robot.frm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.flm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rrm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rlm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    if (Distance > 0){
        
    
    // Set drive power

    
    while(opModeIsActive()&&(-robot.frm.getCurrentPosition()<Distance) && (robot.flm.getCurrentPosition()<Distance) && (-robot.rrm.getCurrentPosition()<Distance) && (robot.rlm.getCurrentPosition()<Distance)){
if( runtime.seconds() < 1){
    rpower=power * (runtime.seconds()/1);
    lpower=power * (runtime.seconds()/1);
}
else{
    rpower=power;
    lpower=power;
}
        double down = Math.abs(Distance)-Math.abs(robot.frm.getCurrentPosition());
    if(Math.abs(down) <80&&Math.abs(down)>10){
        rpower*=(Math.abs(down)/80);
        lpower*=(Math.abs(down)/80);
    }
        else if(Math.abs(down)<10){
        rpower=.15;
        lpower=.15;
        
    }
    
    robot.frm.setPower(-rpower);
    robot.flm.setPower(lpower);
    robot.rrm.setPower(-rpower);
    robot.rlm.setPower(lpower);
}}

    else if (Distance < 0){
        
    

    
    while(opModeIsActive()&&(-robot.frm.getCurrentPosition()>Distance) && (robot.flm.getCurrentPosition()>Distance) && (robot.rrm.getCurrentPosition()>Distance) && (-robot.rlm.getCurrentPosition()>Distance)){
if( runtime.seconds() < 1){
    rpower=power * (runtime.seconds()/1);
    lpower=power * (runtime.seconds()/1);
}
else{
    rpower=power;
    lpower=power;
}
        double down = Math.abs(Distance)-Math.abs(robot.frm.getCurrentPosition());
    if(Math.abs(down) <80&&Math.abs(down)>10){
        rpower*=(Math.abs(down)/80);
        lpower*=(Math.abs(down)/80);
    }
    else if(Math.abs(down)<10){
        rpower=.15;
        lpower=.15;
        
    }

    robot.frm.setPower(rpower);
    robot.flm.setPower(-lpower);
    robot.rrm.setPower(rpower);
    robot.rlm.setPower(-lpower);
}}


    robot.frm.setPower(0);
    robot.flm.setPower(0);
    robot.rrm.setPower(0);
    robot.rlm.setPower(0);


           
                      
    }
    public void turng(int Distance){
      
                   Distance=Distance *10;
    robot.frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
   
    //set to Run_TO_POSITION OpMode
    robot.frm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.flm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rrm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rlm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    if (Distance > 0){
        
    
    // Set drive power
    robot.frm.setPower(-.3);
    robot.flm.setPower(.3);
    robot.rrm.setPower(-.3);
    robot.rlm.setPower(.3);
    
    while(opModeIsActive()&&(-robot.frm.getCurrentPosition()<Distance) && (robot.flm.getCurrentPosition()<Distance) && (-robot.rrm.getCurrentPosition()<Distance) && (robot.rlm.getCurrentPosition()<Distance)){

}}

    else if (Distance < 0){
        
    
    // Set drive power
    robot.frm.setPower(.3);
    robot.flm.setPower(-.3);
    robot.rrm.setPower(.3);
    robot.rlm.setPower(-.3);
    
    while(opModeIsActive()&&(-robot.frm.getCurrentPosition()>Distance) && (robot.flm.getCurrentPosition()>Distance) && (robot.rrm.getCurrentPosition()>Distance) && (-robot.rlm.getCurrentPosition()>Distance)){

}}


    robot.frm.setPower(0);
    robot.flm.setPower(0);
    robot.rrm.setPower(0);
    robot.rlm.setPower(0);


           
                      
    }
public void resetAngle() {
        Orientation angles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        lastAngles = angles2;
        globalAngle = 0;
    }
    public double getAngle()   {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

    Orientation angles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        
        double deltaAngle = angles2.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles2;

        return globalAngle;
    }
}
