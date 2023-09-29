#include <Wire.h>
#include <SoftWire.h>
#include <AsyncDelay.h>



int x=6, y=6, o=0, fw, rw, lw;

int wall[8][13]=
{
	{1,1,1,1,1,1,1,1,1,1,1,1,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,1,1,1,1,1,1,1,1,1,1,1,1}
};

#define l_trigpin A0
#define l_echopin A1

#define f_trigpin 7
#define f_echopin 6

#define r_trigpin A2
#define r_echopin A3

const int leftSpeed = 9; //enb //means pin 9 on the Arduino controls the speed of left motor
const int rightSpeed = 10; //ena
const int left1 = 12; //left 1 and left 2 control the direction of rotation of left motor
const int left2 = 11;
const int right1 = 5;
const int right2 = 4;

#define forward_change 105
#define turn_change 35
volatile int l_enc_state_change = 0;

#define l_enc_pin 2
#define r_enc_pin 3


int wall_check(int trig, int echo){ //CHECK THE DISTANCE VALUE
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  float distCm = (pulseIn,(echo, HIGH)*0.034/2);
  if(distCm<15){return 1;} else if(distCm>=15) {return 0;}; //CHECK THE DISTANCE

  return 0;

};
 
void l_update_enc() {
  // Read the encoder position
  l_enc_state_change++;
}

void forward()
{
  analogWrite(rightSpeed, 170);
  analogWrite(leftSpeed, 185.3333);
      digitalWrite(right1, HIGH); //the right motor rotates FORWARDS when right1 is HIGH and right2 is LOW
      digitalWrite(right2, LOW);
      digitalWrite(left1, HIGH);
      digitalWrite(left2, LOW);
      delay(1100);
    digitalWrite(right1, LOW);
    digitalWrite(right2, LOW);
    digitalWrite(left1, LOW);
    digitalWrite(left2, LOW);
    delay(1000);
}

void left()
{
  analogWrite(rightSpeed, 130);
  analogWrite(leftSpeed, 132.3333);
  l_enc_state_change=0;
  while(l_enc_state_change < turn_change)
    {
        digitalWrite(right1, HIGH); //the right motor rotates FORWARDS when right1 is HIGH and right2 is LOW
      digitalWrite(right2, LOW);
      digitalWrite(left1, LOW);
      digitalWrite(left2, HIGH);
    }
    digitalWrite(right1, LOW);
    digitalWrite(right2, LOW);
    digitalWrite(left1, LOW);
    digitalWrite(left2, LOW);
    delay(1000);
}

void right()
{
  analogWrite(rightSpeed, 130);
  analogWrite(leftSpeed, 132.3333);
  l_enc_state_change=0;
  while(l_enc_state_change < turn_change)
    {
        digitalWrite(right1, LOW); //the right motor rotates FORWARDS when right1 is HIGH and right2 is LOW
      digitalWrite(right2, HIGH);
      digitalWrite(left1, HIGH);
      digitalWrite(left2, LOW);
    }
    digitalWrite(right1, LOW);
    digitalWrite(right2, LOW);
    digitalWrite(left1, LOW);
    digitalWrite(left2, LOW);
    delay(1000);
}
int config(){ //DONE
  if (o==0) //DONE
  {
    if (lw==1 && fw==0 && rw==0)
    {
      return -1;
    }
    else if (lw==0 && fw==1 && rw==0)
    {
      return -2;
    }
    else if (lw==0 && fw==0 && rw==1)
    {
      return -3;
    }

    else if (lw==0 && fw==1 && rw==1)
    {
      return -7;
    }
    else if (lw==1 && fw==1 && rw==0)
    {
      return -8;
    }
    
    else if (lw==1 && fw==0 && rw==1)
    {
      return -9;
    }
    else if (lw==1 && fw==1 && rw==1)
    {
      return -13;
    }
    else if (lw==0 && fw==0 && rw==0)
    {
      return -15;
    }
  }

  else if (o==1 || o==-3) //DONE
  {
    if (lw==1 && fw==0 && rw==0)
    {
      return -2;
    }
    else if (lw==0 && fw==1 && rw==0)
    {
      return -3;
    }
    else if (lw==0 && fw==0 && rw==1)
    {
      return -4;
    }

    else if (lw==0 && fw==1 && rw==1)
    {
      return -6;
    }
    else if (lw==1 && fw==1 && rw==0)
    {
      return -7;
    }
    
    else if (lw==1 && fw==0 && rw==1)
    {
      return -10;
    }
    else if (lw==1 && fw==1 && rw==1)
    {
      return -12;
    }
    else if (lw==0 && fw==0 && rw==0)
    {
      return -15;
    }
  }
  
  else if (o==2 || o==-2) //DONE
  {
    if (lw==1 && fw==0 && rw==0)
    {
      return -3;
    }
    else if (lw==0 && fw==1 && rw==0)
    {
      return -4;
    }
    else if (lw==0 && fw==0 && rw==1)
    {
      return -1;
    }

    else if (lw==0 && fw==1 && rw==1)
    {
      return -5;
    }
    else if (lw==1 && fw==1 && rw==0)
    {
      return -6;
    }
    
    else if (lw==1 && fw==0 && rw==1)
    {
      return -9;
    }
    else if (lw==1 && fw==1 && rw==1)
    {
      return -11;
    }
    else if (lw==0 && fw==0 && rw==0)
    {
      return -15;
    }
  }
    
  else if (o==3 || o==-1) //DONE
  {
      if (lw==1 && fw==0 && rw==0)
    {
      return -4;
    }
    else if (lw==0 && fw==1 && rw==0)
    {
      return -1;
    }
    else if (lw==0 && fw==0 && rw==1)
    {
      return -2;
    }

    else if (lw==0 && fw==1 && rw==1)
    {
      return -8;
    }
    else if (lw==1 && fw==1 && rw==0)
    {
      return -5;
    }
    
    else if (lw==1 && fw==0 && rw==1)
    {
      return -10;
    }
    else if (lw==1 && fw==1 && rw==1)
    {
      return -14;
    }
    else if (lw==0 && fw==0 && rw==0)
    {
      return -15;
    }
  }

};

void turn()
{
  if(o==0)
  {
    if(rw==1 && fw==1 && lw==1)
    {
    	right();
    	right();
    	o++;
    	o++;
    	y++;
    	forward();
	}
    
	else if(wall[y][x+1]==0 && rw==0)
    {
    	right();
    	o++;
    	x++;
    	forward();
	}
	
	else if(wall[y-1][x]==0 && fw==0)
	{
		y--;
		forward();
	}
	
	else if(wall[y][x-1]==0 && lw==0)
    {
    	left();
    	o--;
    	x--;
    	forward();
	}
	else
	{
		if(rw==0)
		{
			right();
			o++;
			x++;
			forward();
		}
		
		else if(fw==0)
		{
			y--;
			forward();
		}
		
		else
		{
			left();
			o--;
			x--;
			forward();
		}
	}
  }

  else if(o==1 || o==-3)
  {
    if(rw==1 && fw==1 && lw==1)
    {
    	right();
    	right();
    	o++;
    	o++;
    	x--;
    	forward();
	}
    
	else if(wall[y+1][x]==0 && rw==0)
    {
    	right();
    	o++;
    	y++;
    	forward();
	}
	
	else if(wall[y][x+1]==0 && fw==0)
	{
		x++;
		forward();
	}
	
	else if(wall[y-1][x]==0 && lw==0)
    {
    	left();
    	o--;
    	y--;
    	forward();
	}
	else
	{
		if(rw==0)
		{
			right();
			o++;
			y++;
			forward();
		}
		
		else if(fw==0)
		{
			x++;
			forward();
		}
		
		else
		{
			left();
			o--;
			y--;
			forward();
		}
	}
  }

  else if(o==2 || o==-2)
  {
    if(rw==1 && fw==1 && lw==1)
    {
    	right();
    	right();
    	o++;
    	o++;
    	y--;
    	forward();
	}
    
	else if(wall[y][x-1]==0 && rw==0)
    {
    	right();
    	o++;
    	x--;
    	forward();
	}
	
	else if(wall[y+1][x]==0 && fw==0)
	{
		y++;
		forward();
	}
	
	else if(wall[y][x+1]==0 && lw==0)
    {
    	left();
    	o--;
    	x++;
    	forward();
	}
	else
	{
		if(rw==0)
		{
			right();
			o++;
			x--;
			forward();
		}
		
		else if(fw==0)
		{
			y++;
			forward();
		}
		
		else
		{
			left();
			o--;
			x++;
			forward();
		}
	}
  }

  else
  {
    if(rw==1 && fw==1 && lw==1)
    {
    	right();
    	right();
    	o++;
    	o++;
    	x++;
    	forward();
	}
    
	else if(wall[y-1][x]==0 && rw==0)
    {
    	right();
    	o++;
    	y--;
    	forward();
	}
	
	else if(wall[y][x-1]==0 && fw==0)
	{
		x--;
		forward();
	}
	
	else if(wall[y+1][x]==0 && lw==0)
    {
    	left();
    	o--;
    	y++;
    	forward();
	}
	else
	{
		if(rw==0)
		{
			right();
			o++;
			y--;
			forward();
		}
		
		else if(fw==0)
		{
			x--;
			forward();
		}
		
		else
		{
			left();
			o--;
			y++;
			forward();
		}
	}
  }

  o=o%4;
}
void setup() {
  pinMode(l_trigpin, OUTPUT); // Sets the trigPin as an Output
  pinMode(l_echopin, INPUT); // Sets the echoPin as an Input

  pinMode(f_trigpin, OUTPUT); // Sets the trigPin as an Output
  pinMode(f_echopin, INPUT); // Sets the echoPin as an Input

  pinMode(r_trigpin, OUTPUT); // Sets the trigPin as an Output
  pinMode(r_echopin, INPUT); // Sets the echoPin as an Input
  pinMode(left1, OUTPUT);
  pinMode(left2, OUTPUT);
  pinMode(right1, OUTPUT);
  pinMode(right2, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(rightSpeed, OUTPUT);

  analogWrite(rightSpeed, 200);
  analogWrite(leftSpeed, 208.3333);

  attachInterrupt(digitalPinToInterrupt(l_enc_pin), l_update_enc, CHANGE);
  delay(1000);
}
void loop() 
{
  while(1)
	{ 
    	fw=wall_check(f_trigpin,f_echopin);
  		rw=wall_check(r_trigpin,r_echopin);
  		lw=wall_check(l_trigpin,l_echopin);
		if (lw==1 && fw==0 && rw==0)
    	{
    	  	wall[4][1]=-5;
	    }
    	else if (lw==0 && fw==0 && rw==1)
	    {
    	  	wall[4][1]=-6;
    	}

    	else if (lw==0 && fw==1 && rw==1)
    	{
      		wall[4][1]=-12;
    	}
    	else if (lw==1 && fw==1 && rw==0)
    	{
      		wall[4][1]=-14;
    	}
   	 	else if (lw==1 && fw==0 && rw==1)
    	{
      		wall[4][1]=-11;
    	}
    	else if (lw==0 && fw==0 && rw==0)
    	{
      		wall[4][1]=-4;
    	}
   		turn();
  		while(1)
  		{
  			fw=wall_check(f_trigpin,f_echopin);
  		  rw=wall_check(r_trigpin,r_echopin);
  		  lw=wall_check(l_trigpin,l_echopin);
			  wall[y][x]=config();
  			turn();
		}
	
	
  }
}
