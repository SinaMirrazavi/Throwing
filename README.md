# Throwing


The code is straightforward if you know RobotToolkit and Seungsu! Here I will explain  kukafastthrowing as randomthrowing do not work. The target position can be the position of an object streamed to the code, the name of the "obj_root" and defined in line 25, object is  or it can be hard coded, see lines 505 as follow:  

```
    	lTargetPos(0) = -3.4;
    	lTargetPos(1) = -1.200;
    	lTargetPos(2) =  0.0;
```
To estimate the throwing position and the velocity the following algorithm is implemented:

1. The robot  moves to the neighborhood of the predefined throwing position, cThrow.
2. The velocity perpendicular to the end-effector position is calculated.
3. The position of the first joint is modified such that the direction of the velocity will be aligned with the line passes though the target position and the the throwing position.
4. The throwing velocity is calculated by trial and error. The concept is easy: the throwing scenario is simulated and then the error at the target position is calculated. The error is used to modify the throwing velocity; see the following lines:
```
for(int i=0; i<50; i++){
		lKp  *= (1.0 -(lError(0)*0.01));
		lKp  *= (1.0 +(lError(2)*0.01));

		mBallistic->SetPosVel(lPos.Array(), (lVel*lKp).Array());
		lDuration = mBallistic->CalSmallestError(targetPos, lError);

		if( lError.Norm() < 0.001) break;
		cout << "error : " << lError(0) << " " << lError(1) << " " << lError(2) << ", kp " <<  lKp << endl;
	}
```
5. The ready configuration is calculated:
```
 mJointReady = mJointThrow - (lJointsVel*_dt*70.0);
```
6. CDdymic is used to move the robot to the ready position.
7. Third order polynomial motion is used to move the robot from the ready position to the throwing position with the desired velocity. 

#### How to run the code:
In short:

1. job
2. throw

If you think the calculated throwing velocity or direction needs some modification, use 
setym ( modification to the direction) ( modification to the velocity)


