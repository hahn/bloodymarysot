// Strategy.cpp : Defines the entry point for the DLL application.
//
//tes coba 30012012

#include "stdafx.h"
#include "Strategy.h"
#include <stdlib.h>
#include <math.h>
#include <time.h>

BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved
					 )
{
    switch (ul_reason_for_call)
	{
		case DLL_PROCESS_ATTACH:
		case DLL_THREAD_ATTACH:
		case DLL_THREAD_DETACH:
		case DLL_PROCESS_DETACH:
			break;
    }
    return TRUE;
}



const double PI = 3.1415923;
//30012012 nambah home and opp Goal
int homeGoal =0;
int oppGoal = 0;
time_t start,end;
int gState=0;
FILE * filejarak; 

//sampai sini

char myMessage[200]; //big enough???

void PredictBall ( Environment *env );
void Goalie1 ( Robot *robot, Environment *env );
void NearBound2 ( Robot *robot, double vl, double vr, Environment *env );
void Attack2 ( Robot *robot, Environment *env );
void Defend ( Robot *robot, Environment *env, double low, double high );

// by moon at 9/2/2002
void MoonAttack (Robot *robot, Environment *env );
// just for testing to check whether the &env->opponent works or not
void MoonFollowOpponent (  Robot *robot, OpponentRobot *opponent );
//tes nambah 30012012
//tes 070202, strategy untuk attacker
void Attacker( Robot *robot, Environment *env );
void MDefender(Robot *robot, Environment *env);
void cetakJarak(Environment *env);
void FollowBallPelan(Robot *robot, Environment *env);
void Position2( Robot *robot, double x, double y );
void cetak(char *txt);
void PrediksiBola ( Environment *env );
//sampai sini nambahnya
void Velocity ( Robot *robot, int vl, int vr );
void Angle ( Robot *robot, int desired_angle);
void Position( Robot *robot, double x, double y );

extern "C" STRATEGY_API void Create ( Environment *env )
{
	// allocate user data and assign to env->userData
	// eg. env->userData = ( void * ) new MyVariables ();
}

extern "C" STRATEGY_API void Destroy ( Environment *env )
{
	// free any user data created in Create ( Environment * )

	// eg. if ( env->userData != NULL ) delete ( MyVariables * ) env->userData;
}


extern "C" STRATEGY_API void Strategy ( Environment *env )
{

	// the below codes are just for demonstration purpose....don't take this seriously please.
	time(&start);
	gState++;
	filejarak = fopen("C:\\Strategy\\blue\\jarak.txt","w");
	int testInt = 100;

	double pBall= env->currentBall.pos.x;

	switch (env->gameState)
	{
		case 0:
			// default
			//di sini keadaan normal permainan
			//MoonFollowOpponent ( &env->home [1], &env->opponent [4] );
			//MoonFollowOpponent ( &env->home [2], &env->opponent [4] );
			//MoonFollowOpponent ( &env->home [3], &env->opponent [4] );
			MDefender( &env->home[1], env);
			//Attacker( &env->home [4], env );
			///FollowBallPelan(&env->home[2],env);
			MoonAttack ( &env->home [2], env );
			Goalie1 ( &env->home [0], env );
			homeGoal = gState;
			////////////
			//tes gol kecetak ga
			
			
			//FILE * debugfile; 
			//debugfile = fopen("C:\\Strategy\\blue\\debugfile.txt","w"); 
		//	fprintf(debugfile,"goal home: %i\ngoal opp: %i\n",homeGoal, oppGoal);
			
			/*
			for (k=0;k<=4;k++) 
				fprintf(debugfile, "robot: %d x: %f y: %f z: %f \n",
					k, env->opponent[k].pos.x, env->opponent[k].pos.y, 
					env->opponent[k].pos.z); 
			*/
		//	fclose(debugfile); 
			//////////
			//tes sampai sini (30012012)

			break;

		case FREE_BALL:

			// Follow opponent guy
			MoonFollowOpponent ( &env->home [1], &env->opponent [4] );
			MoonFollowOpponent ( &env->home [2], &env->opponent [4] );
			MoonFollowOpponent ( &env->home [3], &env->opponent [4] );

			// attack
			
			//MoonAttack ( &env->home [4], env );

			// Goal keeper
			Goalie1 ( &env->home [0], env );

			// by moon at 24/03/2002
			// below code will not work.... never try....
			//	env->home[0].pos.x = 50;
			//	env->home[0].pos.y = 0;
			//	env->home[0].rotation = 20.0;

			break;

		case PLACE_KICK:
			MoonAttack ( &env->home [2], env );
			break;			
		case PENALTY_KICK:
			switch (env->whosBall)
			{
			case ANYONES_BALL:
				MoonAttack ( &env->home [1], env );
				break;
			case BLUE_BALL:
				MoonAttack ( &env->home [4], env );
				break;
			case YELLOW_BALL:
				MoonAttack ( &env->home [0], env );
				break;
			}
			break;

		case FREE_KICK:

			

			MoonAttack ( &env->home [4], env );

			break;

		case GOAL_KICK:
			MoonAttack ( &env->home [0], env );
			break;
  }
}

void MoonAttack ( Robot *robot, Environment *env )
{
	//Velocity (robot, 127, 127);
	//Angle (robot, 45);
	//PredictBall ( env );
	
	PrediksiBola (env );
	Position(robot, env->predictedBall.pos.x, env->predictedBall.pos.y);
	// Position(robot, 0.0, 0.0);
	cetakJarak(env);
	//nambah.
	double x = env->currentBall.pos.x;
}

//tes, nambah 0702020
void Attacker ( Robot *robot, Environment *env )
{
	double x=0.0;
	x = robot->pos.x;
	/*
	if(x<20) {
		Velocity (robot, -20, -20);
	}else if(x>40){
		Velocity (robot, 10, 10);
	}else if(x>60){
		Velocity (robot, -50, -50);
	}else if(x>80){
		//Velocity (robot, 127, 127);
		Position(robot,15,0);
	}
	*/
	//belajar path planning (14022012)
	Velocity(robot,10,10);
	//MoonFollowOpponent(robot,&env->opponent[1]);

	//lihta posisi lawan 9opp 3 dulu)
	double dx =0, dy =0, de=0, jarak=0;

	dx = abs(robot->pos.x - env->opponent[3].pos.x);
	dy = abs( robot->pos.y - env->opponent[3].pos.y);
	de = sqrt( dx*dx + dy*dy);
	//if (de< 10) { //posisi x robot sudah dekat
		//Angle(robot,45);
	//	Velocity(robot,5,10);
		
	//}else if(de<10 && dy<10) {
	//	Velocity(robot,10,5);
	//}
	////jarak dis ini aja bisa ga ya
	//double jarak=0;
	double j[5], d_e=0 ;//, dx=0,dy=0,de=0, temp;
	for(int i=0;i<5;i++)
	{
		dx = abs(robot->pos.x - env->opponent[i].pos.x);
		dy = abs(robot->pos.y - env->opponent[i].pos.y);
		d_e = sqrt(dx*dx + dy*dy);
		j[i] = d_e;
	}
	//cari jarak terdekat
	jarak = j[0];
	for (int i =1;i<5;i++){
		if(jarak> j[i])
			jarak = j[i];
	}
	if (jarak < 10) { //posisi x robot sudah dekat
		//Angle(robot,45);
		Velocity(robot,5,10);
		
	}

	cetakJarak(env);
	///sampai sini///
	
	FILE * debugfile; 
			debugfile = fopen("C:\\Strategy\\blue\\debugfile.txt","w"); 
			fprintf(debugfile,"nilai x: %f\n",x);
			double d_x=0,d_y=0;
			for(int i=0;i<5;i++){
				d_x = abs(robot->pos.x - env->opponent[i].pos.x);
				d_y = abs(robot->pos.y - env->opponent[i].pos.y);
				d_e = sqrt(d_x*d_x + d_y*d_y);
				fprintf(debugfile,"posisi opp[%i] (%f,%f). jarak: %f\n", i,env->opponent[i].pos.x, env->opponent[i].pos.y, d_e);
			}
			fprintf(debugfile,"posisi home[4] (%f,%f). rotasi: %f\n", env->home[4].pos.x, env->home[4].pos.y, robot->rotation);
			fprintf(debugfile,"\nnilai dx,dy (%f,%f). jarak:%f \n", dx,dy, de);
			fprintf(debugfile,"\njarak terdekat:%f \n", jarak);
			//posisi bola
			fprintf(debugfile,"posisi bola: %f,%f,%f)\n",env->currentBall.pos.x,env->currentBall.pos.y,env->currentBall.pos.z);
			fprintf(debugfile,"posisi robot: %f,%f,%f\n",env->home[2].pos.x,env->home[2].pos.y,env->home[2].pos.z);
			fclose(debugfile); 
	//PredictBall ( env ); //penting untuk update posisi bola tiap cycle
	//Position(robot, env->predictedBall.pos.x, env->predictedBall.pos.y);
	// Position(robot, 0.0, 0.0);
}

//iseng, cari jarak terpendek dari semua opponent untuk tiap robot (kecuali goalie)
//14022012

void cetakJarak(Environment *env){
	int y=0,x= homeGoal;
	double d_x=0,d_y=0,d_e=0;
	for(int i=0;i<5;i++){
		fprintf(filejarak,"Posisi relatif lawan untuk robot %i\n",i);
		for(int j=0;j<5;j++){
			d_x = abs(env->home[i].pos.x - env->opponent[j].pos.x);
			d_y = abs(env->home[i].pos.y - env->opponent[j].pos.y);
			d_e = sqrt(d_x*d_x + d_y*d_y);
			fprintf(filejarak,"posisi opp[%i] (%f,%f). jarak: %f\n", j,env->opponent[j].pos.x, env->opponent[j].pos.y, d_e);
		}
		fprintf(filejarak,"\n\n=============================\n");
	}
	for(int i=1;i<5;i++){
		if(i==2) i++;
		d_x = abs(env->home[i].pos.x - env->home[2].pos.x);
		d_y = abs(env->home[i].pos.y - env->home[2].pos.y);
		d_e = sqrt(d_x*d_x + d_y*d_y);
		fprintf(filejarak,"jarak dari home2 ke home[%i]: %f\n",i,d_e);
	}
	Vector3D pos_cur, pos_pred, pos_last, pos_robot;
	pos_cur = env->currentBall.pos;
	pos_pred = env->predictedBall.pos;
	pos_last = env->lastBall.pos;
	pos_robot = env->home[2].pos;
	double dx=0,dy=0, jarakB_r=0;
	dx = env->currentBall.pos.x - env->lastBall.pos.x;
	dy = env->currentBall.pos.y - env->lastBall.pos.y;
	d_x = abs (env->currentBall.pos.x - pos_robot.x);
	d_y = abs (env->currentBall.pos.y - pos_robot.y);
	jarakB_r = sqrt(d_x*d_x + d_y*d_y);
	//print posisi bola
	fprintf(filejarak,"posisi robot\t: %f,%f\n",pos_robot.x,pos_robot.y);
	fprintf(filejarak,"jarak bola\t: %f\n",jarakB_r);
	fprintf(filejarak,"posisi bola\t: %f,%f\n",pos_cur.x,pos_cur.y);
	fprintf(filejarak,"posisi last\t: %f,%f\n",pos_last.x,pos_last.y);
	fprintf(filejarak,"pred bola\t: %f,%f\n",pos_pred.x,pos_pred.y);
	fprintf(filejarak,"selisih: %f,%f\n",(dx),(dy));
	fprintf(filejarak,"selisih10: %f,%f\n",(30*dx),(30*dy));
	//
	time(&end);
	double tt = difftime(end,start);
	fprintf(filejarak,"time %d\n",tt);
	fprintf(filejarak,"gState: %i, x:%i\n",gState,x);
	fclose(filejarak); 
}

void FollowBallPelan(Robot *robot, Environment *env){
	
	//mengikuti bola tapi pelan pelan saja
	double jarak=0, dx=0,dy=0;
	double vl = robot->velocityLeft;
	double vr = robot->velocityRight;
	
	char *m=new char[10];
	PredictBall(env);
	Position(robot, env->predictedBall.pos.x, env->predictedBall.pos.y);
	dx = abs(robot->pos.x - env->predictedBall.pos.x);
	dx = abs(robot->pos.y - env->predictedBall.pos.y);
	jarak = sqrt(dx*dx + dy*dy); 
	//m = _itoa(jarak,m,10);
	//cetak(m);
	double d_x=0,d_y=0,d_e=0;
	double tmp=100;
	int whichrobot=0;
	for(int i=1;i<5;i++){
		if(env->home[i].pos.x-robot->pos.x==0) i++;
		d_x = abs(env->home[i].pos.x - robot->pos.x);
		d_y = abs(env->home[i].pos.y - robot->pos.y);
		d_e = sqrt(d_x*d_x + d_y*d_y);
		if(tmp>d_e) {
			tmp=d_e;
			whichrobot=i;
		}
	}
	m = _itoa(tmp,m,10);
	cetak(m);
	if(jarak <= 1){
		//hitung jarak kawan. sementara 4 aja minus kiper
		
		Position(robot,env->home[whichrobot].pos.x,env->home[whichrobot].pos.y);
		jarak=1;
	}
	cetakJarak(env);
}
void cetak(char *txt){
	printf("%c",txt);
	FILE * tes; 
	tes = fopen("C:\\Strategy\\blue\\tes.txt","w");
	fprintf(tes,"%s\n\n=============================\n",txt);
	fclose(tes); 
}
void Position2( Robot *robot, double x, double y )
{
	//tes bisa ga kejar bola tapi kecepatannya rendah//16022012
	int desired_angle = 0, theta_e = 0, d_angle = 0, vl, vr, vc = 10;

	double dx, dy, d_e, Ka = 10.0/90.0;
	dx = x - robot->pos.x;//jarak x
	dy = y - robot->pos.y;//jarak y

	d_e = sqrt(dx * dx + dy * dy);//jarak pytagoras
	if (dx == 0 && dy == 0)
		desired_angle = 90;
	else
		desired_angle = (int)(180. / PI * atan2((double)(dy), (double)(dx)));
	theta_e = desired_angle - (int)robot->rotation;
	
	while (theta_e > 180) theta_e -= 360;
	while (theta_e < -180) theta_e += 360;

	if (d_e > 100.) 
		Ka = 17. / 90.;
	else if (d_e > 50)
		Ka = 19. / 90.;
	else if (d_e > 30)
		Ka = 21. / 90.;
	else if (d_e > 20)
		Ka = 23. / 90.;
	else 
		Ka = 25. / 90.;
	
	if (theta_e > 95 || theta_e < -95)
	{
		theta_e += 180;
		
		if (theta_e > 180) 
			theta_e -= 360;
		if (theta_e > 80)
			theta_e = 80;
		if (theta_e < -80)
			theta_e = -80;
		if (d_e < 5.0 && abs(theta_e) < 40)
			Ka = 0.1;
		vr = (int)(-vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) + Ka * theta_e);
		vl = (int)(-vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) - Ka * theta_e);
	}
	
	else if (theta_e < 85 && theta_e > -85)
	{
		if (d_e < 5.0 && abs(theta_e) < 40)
			Ka = 0.1;
		vr = (int)( vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) + Ka * theta_e);
		vl = (int)( vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) - Ka * theta_e);
	}

	else
	{
		vr = (int)(+.17 * theta_e);
		vl = (int)(-.17 * theta_e);
	}

	Velocity(robot, vl, vr);
}



//iseng sampai sini
//tes nambah 30012012
void MDefender (Robot *robot, Environment *env){
	//PredictBall(env);
	//
	double r = robot->rotation;
	if (r>45){
		Angle(&env->home[1],91);
	} else if (r>90){
		Angle(&env->home[1],1);
	} else {
		Angle(&env->home[1],50);
	}
	//
}


//ini nambah gl 22022012
void PrediksiBola ( Environment *env ){
	double dx=0,dy=0, jarak=0,j_x=0,j_y=0;
	dx = env->currentBall.pos.x - env->lastBall.pos.x;
	dy = env->currentBall.pos.y - env->lastBall.pos.y;
	j_x = abs(env->currentBall.pos.x- env->home[2].pos.x); 
	j_y = abs(env->currentBall.pos.y- env->home[2].pos.y);
	jarak = sqrt(j_x*j_x + j_y*j_y);
	if(jarak>15) {
		dx= 40*dx;
		dy= 40*dy;
	} else if(jarak>10){
		dx= 10*dx;
		dy= 10*dy;
	}else {
		dx= 1*dx;
		dy= 1*dy;
	}

	env->predictedBall.pos.x = env->currentBall.pos.x + dx;
	env->predictedBall.pos.y = env->currentBall.pos.y + dy;
	//fclose(filejarak);

}
void MoonFollowOpponent ( Robot *robot, OpponentRobot *opponent )
{
	Position(robot, opponent->pos.x, opponent->pos.y);
}

void Velocity ( Robot *robot, int vl, int vr )
{
	robot->velocityLeft = vl;
	robot->velocityRight = vr;
}

// robot soccer system p329
void Angle ( Robot *robot, int desired_angle)
{
	int theta_e, vl, vr;
	//nambah 30012012
	vl = 0;
	vr = 0;
	theta_e = desired_angle - (int)robot->rotation;
	
	while (theta_e > 180) theta_e -= 360;
	while (theta_e < -180) theta_e += 360;

	if (theta_e < -90) theta_e += 180;
	
	else if (theta_e > 90) theta_e -= 180;

	if (abs(theta_e) > 50) 
	{
		vl = (int)(-9./90.0 * (double) theta_e);
		vr = (int)(9./90.0 * (double)theta_e);
	}
	else if (abs(theta_e) > 20)
	{
		vl = (int)(-11.0/90.0 * (double)theta_e);
		vr = (int)(11.0/90.0 * (double)theta_e);
	}
	Velocity (robot, vl, vr);
}

void Position( Robot *robot, double x, double y )
{
	int desired_angle = 0, theta_e = 0, d_angle = 0, vl, vr, vc = 70;

	double dx, dy, d_e, Ka = 10.0/90.0;
	dx = x - robot->pos.x;
	dy = y - robot->pos.y;

	d_e = sqrt(dx * dx + dy * dy);
	if (dx == 0 && dy == 0)
		desired_angle = 90;
	else
		desired_angle = (int)(180. / PI * atan2((double)(dy), (double)(dx)));
	theta_e = desired_angle - (int)robot->rotation;
	
	while (theta_e > 180) theta_e -= 360;
	while (theta_e < -180) theta_e += 360;

	if (d_e > 100.) 
		Ka = 17. / 90.;
	else if (d_e > 50)
		Ka = 19. / 90.;
	else if (d_e > 30)
		Ka = 21. / 90.;
	else if (d_e > 20)
		Ka = 23. / 90.;
	else 
		Ka = 25. / 90.;
	
	if (theta_e > 95 || theta_e < -95)
	{
		theta_e += 180;
		
		if (theta_e > 180) 
			theta_e -= 360;
		if (theta_e > 80)
			theta_e = 80;
		if (theta_e < -80)
			theta_e = -80;
		if (d_e < 5.0 && abs(theta_e) < 40)
			Ka = 0.1;
		vr = (int)(-vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) + Ka * theta_e);
		vl = (int)(-vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) - Ka * theta_e);
	}
	
	else if (theta_e < 85 && theta_e > -85)
	{
		if (d_e < 5.0 && abs(theta_e) < 40)
			Ka = 0.1;
		vr = (int)( vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) + Ka * theta_e);
		vl = (int)( vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) - Ka * theta_e);
	}

	else
	{
		vr = (int)(+.17 * theta_e);
		vl = (int)(-.17 * theta_e);
	}

	Velocity(robot, vl, vr);
}


void PredictBall ( Environment *env )
{
	double dx = env->currentBall.pos.x - env->lastBall.pos.x;
	double dy = env->currentBall.pos.y - env->lastBall.pos.y;
	//env->predictedBall.pos.x = env->currentBall.pos.x + dx;
	//env->predictedBall.pos.y = env->currentBall.pos.y + dy;
	//nambah jadina kumaha nya. aslinya yang di atas
	
	env->predictedBall.pos.x = env->currentBall.pos.x + (dx);
	env->predictedBall.pos.y = env->currentBall.pos.y + (dy);
	

}

void Goalie1 ( Robot *robot, Environment *env )
{
	double velocityLeft = 0, velocityRight = 0;
	
	double Tx = env->goalBounds.right - env->currentBall.pos.x;
	double Ty = env->fieldBounds.top - env->currentBall.pos.y;

	double Ax = env->goalBounds.right - robot->pos.x;
	double Ay = env->fieldBounds.top - robot->pos.y;

	if ( Ay > Ty + 0.9 && Ay > 27 )
	{
		velocityLeft = -100;
		velocityRight = -100;
	}

	if ( Ay > Ty - 0.9 && Ay < 43 )
	{
		velocityLeft = 100;
		velocityRight = 100;
	}

	if ( Ay < 27 )
	{
		velocityLeft = 100;
		velocityRight = 100;
	}

	if ( Ay > 43 )
	{
		velocityLeft = -100;
		velocityRight = -100;
	}

	double Tr = robot->rotation;
	if ( Tr < 0.001 )
		Tr = Tr + 360;
	if ( Tr > 360.001 )
		Tr = Tr - 360;
	if ( Tr > 270.5 )
		velocityRight = velocityRight + fabs ( Tr - 270 );
	else if ( Tr < 269.5 )
		velocityLeft = velocityLeft + fabs ( Tr - 270 );

	robot->velocityLeft = velocityLeft;
	robot->velocityRight = velocityRight;
}



void Attack2 ( Robot *robot, Environment *env )
{
	Vector3D t = env->currentBall.pos;
	double r = robot->rotation;
	if ( r < 0 ) r += 360;
	if ( r > 360 ) r -= 360;
	double vl = 0, vr = 0;

	if ( t.y > env->fieldBounds.top - 2.5 ) t.y = env->fieldBounds.top - 2.5;
	if ( t.y < env->fieldBounds.bottom + 2.5 ) t.y = env->fieldBounds.bottom + 2.5;
	if ( t.x > env->fieldBounds.right - 3 ) t.x = env->fieldBounds.right - 3;
	if ( t.x < env->fieldBounds.left + 3 ) t.x = env->fieldBounds.left + 3;

	double dx = robot->pos.x - t.x;
	double dy = robot->pos.y - t.y;

	double dxAdjusted = dx;
	double angleToPoint = 0;

	if ( fabs ( robot->pos.y - t.y ) > 7 || t.x > robot->pos.x )
		dxAdjusted -= 5;

	if ( dxAdjusted == 0 )
	{
		if ( dy > 0 )
			angleToPoint = 270;
		else
			angleToPoint = 90;
	}
	else if ( dy == 0 )
	{
		if ( dxAdjusted > 0 )
			angleToPoint = 360;
		else
			angleToPoint = 180;
		
	}
	else
		angleToPoint = atan ( fabs ( dy / dx ) ) * 180.0 / PI;

	if ( dxAdjusted > 0 )
	{
		if ( dy > 0 )
			angleToPoint -= 180;
		else if ( dy < 0 )
			angleToPoint = 180 - angleToPoint;
	}
	if ( dxAdjusted < 0 )
	{
		if ( dy > 0 )
			angleToPoint = - angleToPoint;
		else if ( dy < 0 )
			angleToPoint = 90 - angleToPoint;
	}

	if ( angleToPoint < 0 ) angleToPoint = angleToPoint + 360;
	if ( angleToPoint > 360 ) angleToPoint = angleToPoint - 360;
	if ( angleToPoint > 360 ) angleToPoint = angleToPoint - 360;

	double c = r;

	double angleDiff = fabs ( r - angleToPoint );

	if ( angleDiff < 40 )
	{
		vl = 100;
		vr = 100;
		if ( c > angleToPoint )
			vl -= 10;
		if ( c < angleToPoint )
			vr -= 10;
	}
	else
	{
		if ( r > angleToPoint )
		{
			if ( angleDiff > 180 )
				vl += 360 - angleDiff;
			else
				vr += angleDiff;
		}
		if ( r < angleToPoint )
		{
			if ( angleDiff > 180 )
				vr += 360 - angleDiff;
			else
				vl += angleDiff;
		}
	}

	NearBound2 ( robot, vl, vr, env );
}

void NearBound2 ( Robot *robot, double vl, double vr, Environment *env )
{
	//Vector3D t = env->currentBall.pos;

	Vector3D a = robot->pos;
	double r = robot->rotation;

	if ( a.y > env->fieldBounds.top - 15 && r > 45 && r < 130 )
	{
		if ( vl > 0 )
			vl /= 3;
		if ( vr > 0 )
			vr /= 3;
	}

	if ( a.y < env->fieldBounds.bottom + 15 && r < -45 && r > -130 )
	{
		if ( vl > 0 ) vl /= 3;
		if ( vr > 0 ) vr /= 3;
	}

	if ( a.x > env->fieldBounds.right - 10 )
	{
		if ( vl > 0 )
			vl /= 2;
		if ( vr > 0 )
			vr /= 2;
	}

	if ( a.x < env->fieldBounds.left + 10 )
	{
		if ( vl > 0 )
			vl /= 2;
		if ( vr > 0 )
			vr /= 2;
	}

	robot->velocityLeft = vl;
	robot->velocityRight = vr;
}

void Defend ( Robot *robot, Environment *env, double low, double high )
{
	double vl = 0, vr = 0;
	Vector3D z = env->currentBall.pos;

	double Tx = env->goalBounds.right - z.x;
	double Ty = env->fieldBounds.top - z.y;
	Vector3D a = robot->pos;
	a.x = env->goalBounds.right - a.x;
	a.y = env->fieldBounds.top - a.y;

	if ( a.y > Ty + 0.9 && a.y > low )
	{
		vl = -100;
		vr = -100;
	}
	if ( a.y < Ty - 0.9 && a.y < high )
	{
		vl = 100;
		vr = 100;
	}
	if ( a.y < low )
	{
		vl = 100;
		vr = 100;
	}
	if ( a.y > high )
	{
		vl = -100;
		vr = -100;
	}

	double Tr = robot->rotation;

	if ( Tr < 0.001 )
		Tr += 360;
	if ( Tr > 360.001 )
		Tr -= 360;
	if ( Tr > 270.5 )
		vr += fabs ( Tr - 270 );
	else if ( Tr < 269.5 )
		vl += fabs ( Tr - 270 );

	NearBound2 ( robot, vl ,vr, env );
}

