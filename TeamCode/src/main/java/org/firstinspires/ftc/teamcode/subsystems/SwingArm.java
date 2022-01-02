package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SwingArm {
    protected HardwareMap hwMap;
    ElapsedTime timer;
    double curr, target, increament;
    boolean firstCall;
    Servo hand,  wrist, shoulder;
    CRServo cwrist;
    double hIntake = 0.68; //0.6; //hold position 0.3 intake
    double hHome = 0.85;//home position too much
    double hRelease3 = 0.0; //depend on the wrist position
    double hRelease2 = 0.15;
    double hRelease1 = 0.2;
    double hHold3 = 0.4;
    double hHold2 = 0.35;
    double hHold1 = 0.5;
    double hAHold = 0.5;

    double wInit = 0.5;  //maybe the position when turn on the power
    double wHome= 0.72; // lowest
    double wIntake = 0.68; // 0.65;
    double wMin = 0.50; // min high to swing
    double wLevel1 = 0.38; //1 lowest;
    double wLevel2 = 0.31; // up 0 position
    double wLevel3 = 0.25;
    double wALevel1 = 0.38;
    double wLift = wLevel2;
    double wTeam = 0; //team element up 0 pposition

    double sMiddle = 0.57; // middle   - more;
    double sLeft = 0.22;//lower
    double sRight = 0.85; // no wire now 0.9;

    boolean busy = false;
    boolean stop = false;

    double currH, currW, currS;

    public enum HandState {
        HOLD,
        PICKUP,

    }
    public enum WristState{
        HOME,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        TEAM
    }
    public enum ShoulderState {
        LEFT,
        RIGHT,
        MIDDLE
    }
    public enum ArmState {
        HOME,
        INTAKE,
        LIFT,
        DELIVER
    }
    HandState hstate;
    WristState wstate;
    ShoulderState sstate;
    ArmState astate;

    public SwingArm (HardwareMap hwMap){
        this.hwMap = hwMap;
    }

    public void init() {
        hand = hwMap.get(Servo.class, "hand");
        wrist = hwMap.get(Servo.class, "wrist");
        cwrist = hwMap.get(CRServo.class, "cwrist");
        shoulder = hwMap.get(Servo.class, "shoulder");
        astate = ArmState.HOME;
        timer = new ElapsedTime();
        firstCall = true;
        stop = false;
        if (wrist.getPosition() < 0.1)  wrist.setPosition(wInit); //when turn the power on the first time
        //busy = true;
        //slowHome();
    }

    public double getHandPos() {return hand.getPosition(); }
    public double getWristPos() {return wrist.getPosition();}
    public double getCWristPos() {return cwrist.getPower();}
    public double getShoulderPos() {return shoulder.getPosition(); }

    public void save() {
        currH = getHandPos();
        currW = getWristPos();
        currS = getShoulderPos();
    }

    public void update(double hPos, double wPos, double sPos) {
        if (stop) return;
        hand.setPosition(hPos);
        wrist.setPosition(wPos);
        shoulder.setPosition(sPos);
    }

    public boolean isBusy() {return busy;}

    public void setCwrist(double power) {
        cwrist.setPower(power);
    }

    public void slowHome() {
        if (firstCall) {
            firstCall = false;
            timer.reset();
            curr = getWristPos();
            return;
        }
        if (timer.milliseconds() >=  50) {
            if (curr  <  wHome) curr += 0.005;
            else curr -= 0.005;
            wrist.setPosition(curr);
            //update(getHandPos(), curr, getShoulderPos());
            timer.reset();
        }
        if ( curr <= wLevel3 || curr >= wHome) {
            wrist.setPosition(wHome);
            //update(getHandPos(), wHome, getShoulderPos());
            firstCall = true;
            busy = false;
        }

        hstate = HandState.HOLD;
        wstate = WristState.HOME;
        sstate = ShoulderState.MIDDLE;
        astate = ArmState.HOME;
    }

    public void slowLift(double increament) {
        if (firstCall) {
            firstCall = false;
            timer.reset();
            curr = getWristPos();
            return;
        }
        if (timer.milliseconds() >=  50) {
            curr += increament;
            if (curr >= wHome || curr <= wLevel3) return; //reach the min and max
            update(getHandPos(), curr, getShoulderPos());
            timer.reset();
            if (curr < wMin) astate = ArmState.LIFT;
        }
        /*
        if (Math.abs(curr - wALevel1) <= 0.01 || curr <= 0.005 || curr >= 0.995) {
            update(getHandPos(), wALevel1, getShoulderPos());
            firstCall = true;
            busy = false;
        }
        */


    }

    public void home(){
        if (astate == ArmState.DELIVER ) {
            return;
        }
        update(hHome, wHome, sMiddle);
        hstate = HandState.HOLD;
        wstate = WristState.HOME;
        sstate = ShoulderState.MIDDLE;
        astate = ArmState.HOME;
    }

    public void intake() {
        if (astate != ArmState.HOME) return;
        update(hIntake, wHome, sMiddle);
        hstate = HandState.PICKUP;
        wstate = WristState.HOME;
        sstate = ShoulderState.MIDDLE;
        astate = ArmState.INTAKE;
    }

    public void adjust(double amount) {
        if (astate == ArmState.INTAKE) {
            if (amount < -0.01) update(hIntake-0.02, wIntake-0.02, sMiddle);
            else update(hIntake+0.05, wIntake, sMiddle);
        }

    }

    public void slowHand(double increment) {

        if (firstCall) {
            firstCall = false;
            timer.reset();
            curr = getHandPos();
            return;
        }
        if (timer.milliseconds() >=  50) {
            curr += increment;
            update(curr, getWristPos(), getShoulderPos());
            timer.reset();
            if (curr <= hIntake) astate = ArmState.INTAKE;
        }
        /*
        if (Math.abs(curr - wALevel1) <= 0.01 || curr <= 0.005 || curr >= 0.995) {
            update(getHandPos(), wALevel1, getShoulderPos());
            firstCall = true;
            busy = false;
        }
        */


    }


    public void lift(int level) {
        if (level == -1) { // lift for swing
            update(hHome, wLift, sMiddle);
            astate = ArmState.LIFT;
            return;
        }
        else if (level == 1) {
            wstate = WristState.LEVEL1;
            update(hHold1, wLevel1, shoulder.getPosition());
        }
        else if (level == 2) {
            wstate = WristState.LEVEL2;
            update(hHold2 , wLevel2, shoulder.getPosition());
        }
        else if (level == 3) {
            wstate = WristState.LEVEL3;
            update(hHold3 , wLevel3, shoulder.getPosition());
        }
        else if (level == 4) {
            update(hand.getPosition(), wTeam, shoulder.getPosition());
        }
        else if (level == 0) {
            update(hand.getPosition(), wHome, shoulder.getPosition());
        }
        else if(level == 11){
            wstate = WristState.LEVEL1;
            update(hAHold, wALevel1, shoulder.getPosition());
        }
        else if (level == -2) {
            update(hHome-.3, wLift, sMiddle);
            astate = ArmState.LIFT;
            return;
        }
        astate = ArmState.DELIVER;
    }

    public void center() {
        if (astate != ArmState.LIFT) return;
        update (hand.getPosition(), wrist.getPosition(), sMiddle);
        sstate = ShoulderState.LEFT;
    }

    public void left() {
        if (astate != ArmState.LIFT) return;
        update (hand.getPosition(), wrist.getPosition(), sLeft);
        sstate = ShoulderState.LEFT;
    }

    public void right() {//change because of cable 12/12
        if (astate != ArmState.LIFT) return;
        update (hand.getPosition(), wrist.getPosition(), sRight);
        sstate = ShoulderState.RIGHT;
    }

    public void deliver3 () {
        lift(3);
    }

    public void deliver1() {
        lift(1);
    }

    public void deliver2() {
        lift(2);
    }

    public void autoDeliver1() {
        lift(11);
    }

    public void release() {
        if (wstate == WristState.LEVEL3) hand.setPosition(hRelease3);
        else if (wstate == WristState.LEVEL2) hand.setPosition(hRelease2);
        else if (wstate == WristState.LEVEL1) hand.setPosition(hRelease1);
    }
    
    public void retrieve(){
        if (wstate == WristState.LEVEL3) hand.setPosition(hHold3);
        if (wstate == WristState.LEVEL2) hand.setPosition(hHold2);
        if (wstate == WristState.LEVEL1) hand.setPosition(hHold1);
    }

    public void test(int indicator, double amount) {
        double hpos = hand.getPosition();
        double wpos = wrist.getPosition();
        double spos = shoulder.getPosition();

        if (indicator == 1) {
            //test hand position
            hand.setPosition(hHome + amount);
        }
        if (indicator == 2) {
            //test wrist position
            wrist.setPosition(wHome + amount);
        }
        if (indicator == 3) {
            //test shoulder position
            shoulder.setPosition(sMiddle + amount);
        }
    }

    public boolean outOfRange() {
        double h = getHandPos();
        double w = getWristPos();
        double s = getShoulderPos();
        return (h < 0.005 || h > 0.995 || w < 0.005 || w > 0.995 || s < 0.005 || s > 0.995);
    }

    public void wMove(double change) {
        if (outOfRange()) return;
        if (change < 0) { //up
            wrist.setPosition(currW * (1 + change));
        }
        else {
            wrist.setPosition(currW * (1 - change));
        }
    }
    public void slowSetPostion(double end, double increment, int cycle )  {
        if (outOfRange()) return;
        if (Math.abs(curr - end) <= Math.abs(increment)) {
            firstCall = true;
            busy = false;
            return;
        }
        if (firstCall) {
            firstCall = false;
            timer.reset();
            busy = true;
            return;
        }
        if (timer.milliseconds() >= cycle) {
            curr += increment;
            wrist.setPosition(curr);
            timer.reset();
        }
    }

    public void stop() {
        stop = true;
        wrist.close();
    }

    public void start() {
        stop = false;
        firstCall = true;

    }

    public void move (Arm3D pos, double increament, int ms) {
        if (stop) return;
        /*if (to == 0) astate = ArmState.HOME;
        else if (to == 1 ) {
            if (astate != ArmState.HOME) return; // intake state can only from home state
            astate = ArmState.INTAKE;
        }
        else if ((to == 2 || to == 3) && astate != ArmState.INTAKE )  return;
*/
        if (firstCall) {
            firstCall = false;
            timer.reset();
            curr3D = new Arm3D(hand.getPosition(), wrist.getPosition(), shoulder.getPosition());
            increament3D = new Arm3D (0.01, 0.01, 0.01) ;
            if (curr3D.handP > pos.handP) increament3D.handP = -0.01;
            else if (curr3D.handP == pos.handP) increament3D.handP = 0;
            if (curr3D.wristP > pos.wristP) increament3D.wristP = -0.01;
            else if (curr3D.wristP == pos.wristP) increament3D.wristP = 0;
            if (curr3D.shoulderP > pos.shoulderP) increament3D.shoulderP = -0.01;
            else if (curr3D.shoulderP == pos.shoulderP) increament3D.shoulderP = 0;
            return;
        }
        if (timer.milliseconds() >=  ms) {
            curr3D.add(increament3D);
            update(curr3D.handP, curr3D.wristP, curr3D.shoulderP);
            timer.reset();
        }
        if (curr3D.reached(pos)) {
            update(pos.handP, pos.wristP, pos.shoulderP);
            firstCall = true;
            busy = false;
        }
    }

    Arm3D curr3D, increament3D;

    Arm3D[] positions = {
            new Arm3D(0.85, 0.71, 0.57),  // 0  home
            new Arm3D(0.68, 0.71, 0.57),  // 1  intake down
            new Arm3D(0.70, 0.71, 0.57),  // 2  intake flat
            new Arm3D(0.70, 0.73, 0.57),  // 3  intake hold
            new Arm3D(0.75, 0.50, 0.57),  // 4  lift
            new Arm3D(0.40, 0.25, 0.22),  // 6  left level 3 hold
            new Arm3D(0.35, 0.31, 0.22),  // 7  left level 2 hold
            new Arm3D(0.50, 0.38, 0.22),  // 8  left level 1 hold
            new Arm3D(0.00, 0.25, 0.22),  // 9  left level 3 release
            new Arm3D(0.15, 0.31, 0.22),  // 10 left level 2 release
            new Arm3D(0.20, 0.38, 0.22),  // 11 left level 1 release
            new Arm3D(0.75, 0.50, 0.85),  // 6 right

    };

    class Arm3D {
        double handP;
        double wristP;
        double shoulderP;

        public Arm3D(double h, double w, double s) {
            handP = h;
            wristP = w;
            shoulderP = s;
        }
        public String toString() {
            return handP + ", " + wristP + ", " + shoulderP;
        }
        public void add(Arm3D a) {
            handP += a.handP;
            wristP += a.wristP;
            shoulderP += a.shoulderP;
        }
        public boolean reached(Arm3D target) {
            return  Math.abs(target.handP - handP) <= 0.02 &&
                    Math.abs(target.wristP - wristP) <= 0.02 &&
                    Math.abs(target.shoulderP - shoulderP) <= 0.02;
        }
    }
}

