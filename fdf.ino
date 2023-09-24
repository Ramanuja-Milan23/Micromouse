
#include <Wire.h>
#include <SoftWire.h>
#include <AsyncDelay.h>
#include <cppQueue.h>



// Motor A connections
#define enA 9
#define lwf 11 //left wheel forward in1
#define lwb 12 //left wheel backward in2

// Motor B connections
#define enB 10
#define rwf 4 //right wheel forward in3
#define rwb 5 //right wheel backward in4

#define l_trigPin A0
#define l_echoPin A1

#define f_trigPin 7
#define f_echoPin 6

#define r_trigPin A2
#define r_echoPin A3

#define l_encoder 1
#define r_encoder 2
#define SIZE 65

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
double z;
double initial_angle;

int arr[SIZE];
int arr1[SIZE];
int Rear = - 1, Front = - 1, top=-1, o=0;
int x = 1, y = 9;

int Wall[11][9]=
{
  {9,9,9,9,9,9,9,9,9},
  {9,7,6,5,4,5,6,7,9},
  {9,6,5,4,3,4,5,6,9},
  {9,5,4,3,2,3,4,5,9},
  {9,4,3,2,1,2,3,4,9},
  {9,3,2,1,0,1,2,3,9},
  {9,4,3,2,1,2,3,4,9},
  {9,5,4,3,2,3,4,5,9},
  {9,6,5,4,3,4,5,6,9},
  {9,7,6,5,4,5,6,7,9},
  {9,9,9,9,9,9,9,9,9}
};

int m[11][9]=
{
  {-1,-1,-1,-1,-1,-1,-1,-1,-1},
  {-1,64,64,64,64,64,64,64,-1},
  {-1,64,64,64,64,64,64,64,-1},
  {-1,64,64,64,64,64,64,64,-1},
  {-1,64,64,64,64,64,64,64,-1},
  {-1,64,64,64,0 ,64,64,64,-1},
  {-1,64,64,64,64,64,64,64,-1},
  {-1,64,64,64,64,64,64,64,-1},
  {-1,64,64,64,64,64,64,64,-1},
  {-1,64,64,64,64,64,64,64,-1},
  {-1,-1,-1,-1,-1,-1,-1,-1,-1}

};










/*
void turn(o); //DONE
void enqueue(x); //DONE
int dequeue(); //DONE
void push(x); //DONE
int pop(); //DONE
*/

float wall_check(int trig, int echo); //CHECK THE DISTANCE VALUE

void forward() {
  int last_state, state, counter=0;

  last_state = digitalRead(l_encoder);
  digitalWrite(lwf, HIGH);
  digitalWrite(rwf, HIGH);
  digitalWrite(lwb, LOW);
  digitalWrite(rwb, LOW);
  while (counter<60)
  {
    state = digitalRead(l_encoder);
    if (state != last_state)
    {
      counter++;
      last_state = state;
    }
  }
  digitalWrite(lwf, LOW);
  digitalWrite(rwf, LOW);
  digitalWrite(lwb, LOW);
  digitalWrite(rwb, LOW);
  return;
};

double angle() { // returns [0,360)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);

  z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
  if (z==360) {return 0;}
  Wire.write(0);
  Wire.endTransmission(true);
  return z;
};

void left() {//DONE
  double b_angle = angle();

  if (b_angle<270){
    while (angle()!=(b_angle+90))
    {
      digitalWrite(rwf, HIGH);
      digitalWrite(lwb, HIGH);
    }
    digitalWrite(rwf, LOW);
    digitalWrite(lwb, LOW);
  }
  else if (b_angle>270){
    while (angle()!=(b_angle+90-360) )
    {
      digitalWrite(rwf, HIGH);
      digitalWrite(lwb, HIGH);
    }
    digitalWrite(rwf, LOW);
    digitalWrite(lwb, LOW);
  }
  else if (b_angle==270){
    while (angle()!=0)
    {
      digitalWrite(rwf, HIGH);
      digitalWrite(lwb, HIGH);
    }
    digitalWrite(rwf, LOW);
    digitalWrite(lwb, LOW);
  }
  return;
};

void right(){//DONE

  double b_angle = angle();

  if (b_angle>90){
    while (angle()!=(b_angle-90))
    {
      digitalWrite(lwf, HIGH);
      digitalWrite(rwb, HIGH);
    }
    digitalWrite(lwf, LOW);
    digitalWrite(rwb, LOW);
  }
  else if (b_angle<90){
    while (angle()!=(b_angle-90+360) )
    {
      digitalWrite(lwf, HIGH);
      digitalWrite(rwb, HIGH);
    }
    digitalWrite(lwf, LOW);
    digitalWrite(rwb, LOW);
  }
  else if (b_angle==90){
    while (angle()!=0)
    {
      digitalWrite(lwf, HIGH);
      digitalWrite(rwb, HIGH);
    }
    digitalWrite(lwf, LOW);
    digitalWrite(rwb, LOW);
  }
  return;
};

int orient(){ //0 for original then right turn 1 then right turn 2 then right turn 3 //DONE

  double b_angle = angle();
  if (initial_angle == 0) //DONE
  {
    if (45<=b_angle && b_angle<135)
    {
      return 1;
    }
    else if (135<=b_angle && b_angle<225)
    {
      return 2;
    }
    else if (225<=b_angle && b_angle<315)
    {
      return 3;
    }
    else
    {
      return 0;
    }
  }
  else if (initial_angle<45) //DONE
  {
      if (initial_angle+45<=b_angle && b_angle<initial_angle+135)
      {
        return 1;
      }
      else if (initial_angle+135<=b_angle && b_angle<initial_angle+225)
      {
        return 2;
      }
      else if (initial_angle+225<=b_angle && b_angle<initial_angle+315)
      {
        return 3;
      }
      else
      {
        return 0;
      }
    
  }
  else if (initial_angle == 45) //DONE
  {
    if (0<=b_angle && b_angle<90)
    {
      return 0;
    }
    else if (90<=b_angle && b_angle<180)
    {
      return 1;
    }
    else if (180<=b_angle && b_angle<270)
    {
      return 2;
    }
    else if (270<=b_angle)
    {
      return 3;
    }
  }
  else if (45<initial_angle && initial_angle<90) //DONE
  {
    if (initial_angle-45<=b_angle && b_angle<initial_angle+45)
      {
        return 0;
      }
      else if (initial_angle+45<=b_angle && b_angle<initial_angle+135)
      {
        return 1;
      }
      else if (initial_angle+135<=b_angle && b_angle<initial_angle+225)
      {
        return 2;
      }
      else 
      {
        return 3;
      }
    
  }
  else if (initial_angle == 90) //DONE
  {
    if (45<=b_angle && b_angle<135)
    {
      return 0;
    }
    else if (135<=b_angle && b_angle<225)
    {
      return 1;
    }
    else if (225<=b_angle && b_angle<315)
    {
      return 2;
    }
    else
    {
      return 3;
    }
  }
  else if (90<initial_angle && initial_angle<135) //DONE
  {
    if (initial_angle-45<=b_angle && b_angle<initial_angle+45)
      {
        return 0;
      }
      else if (initial_angle+45<=b_angle && b_angle<initial_angle+135)
      {
        return 1;
      }
      else if (initial_angle+135<=b_angle && b_angle<initial_angle+225)
      {
        return 2;
      }
      else 
      {
        return 3;
      }
  }
  else if (initial_angle == 135) //DONE
  {
    if (0<=b_angle && b_angle<90)
    {
      return 3;
    }
    else if (90<=b_angle && b_angle<180)
    {
      return 0;
    }
    else if (180<=b_angle && b_angle<270)
    {
      return 1;
    }
    else if (270<=b_angle)
    {
      return 2;
    }
  }
  else if (135<initial_angle && initial_angle<180) //DONE
  {
    if (initial_angle-135<=b_angle && b_angle<initial_angle-45)
      {
        return 3;
      }
      else if (initial_angle-45<=b_angle && b_angle<initial_angle+45)
      {
        return 0;
      }
      else if (initial_angle+45<=b_angle && b_angle<initial_angle+135)
      {
        return 1;
      }
      else 
      {
        return 2;
      }
  }
  else if (initial_angle == 180) //DONE
  {
    if (135<=b_angle && b_angle<225)
    {
      return 0;
    }
    else if (225<=b_angle && b_angle<315)
    {
      return 1;
    }
    else if (45<=b_angle && b_angle<135)
    {
      return 3;
    }
    else
    {
      return 2;
    }
  }
  else if (180<initial_angle && initial_angle<225) //DONE
  {
    if (initial_angle-45<=b_angle && b_angle<initial_angle+45)
      {
        return 0;
      }
      else if (initial_angle+45<=b_angle && b_angle<initial_angle+135)
      {
        return 1;
      }
      else if (initial_angle-135<=b_angle && b_angle<initial_angle-45)
      {
        return 3;
      }
      else 
      {
        return 2;
      }
  }
  else if (initial_angle == 225) //DONE
  {
    if (0<=b_angle && b_angle<90)
    {
      return 2;
    }
    else if (90<=b_angle && b_angle<180)
    {
      return 3;
    }
    else if (180<=b_angle && b_angle<270)
    {
      return 0;
    }
    else if (270<=b_angle)
    {
      return 1;
    }
  }
  else if (225<initial_angle && initial_angle<270) //DONE
  {
    if (initial_angle-45<=b_angle && b_angle<initial_angle+45)
      {
        return 0;
      }
      else if (initial_angle-135<=b_angle && b_angle<initial_angle-45)
      {
        return 3;
      }
      else if (initial_angle-225<=b_angle && b_angle<initial_angle-135)
      {
        return 2;
      }
      else 
      {
        return 1;
      }
  }
  else if (initial_angle == 270) //DONE
  {
    if (135<=b_angle && b_angle<225)
    {
      return 3;
    }
    else if (225<=b_angle && b_angle<315)
    {
      return 0;
    }
    else if (45<=b_angle && b_angle<135)
    {
      return 2;
    }
    else
    {
      return 1;
    }
  }
  else if (270<initial_angle && initial_angle<315) //DONE
  {
    if (initial_angle-45<=b_angle && b_angle<initial_angle+45)
      {
        return 0;
      }
      else if (initial_angle-135<=b_angle && b_angle<initial_angle-45)
      {
        return 3;
      }
      else if (initial_angle-225<=b_angle && b_angle<initial_angle-135)
      {
        return 2;
      }
      else 
      {
        return 1;
      }
  }
  else if (initial_angle == 315) //DONE
  {
    if (0<=b_angle && b_angle<90)
    {
      return 1;
    }
    else if (90<=b_angle && b_angle<180)
    {
      return 2;
    }
    else if (180<=b_angle && b_angle<270)
    {
      return 3;
    }
    else if (270<=b_angle)
    {
      return 0;
    }
  }
  else if (315<initial_angle && initial_angle<360)
  {
    if (initial_angle-135<=b_angle && b_angle<initial_angle-45)
      {
        return 3;
      }
      else if (initial_angle-225<=b_angle && b_angle<initial_angle-135)
      {
        return 2;
      }
      else if (initial_angle-315<=b_angle && b_angle<initial_angle-225)
      {
        return 1;
      }
      else 
      {
        return 0;
      }
  }
  
};

int config(){ //DONE

  int orien=orient();
  int lw=wall_check(l_trigPin, l_echoPin);
  int fw=wall_check(f_trigPin, f_echoPin);
  int rw=wall_check(r_trigPin, r_echoPin);
  if (orien==0) //DONE
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

  else if (orien==1) //DONE
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
  
  else if (orien==2) //DONE
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
    
  else if (orien==3) //DONE
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

float wall_check(int trig, int echo){ //CHECK THE DISTANCE VALUE

  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  float distCm = (pulseIn(echo, HIGH)*0.034/2);
  if(distCm<30){return 1;} else if(distCm>=30) {return 0;}; //CHECK THE DISTANCE
  return 0;
};
 




void enqueue( int n)
{
  Rear++;
  arr[Rear] = n;
  return;
};

void push(int n)
{
  top++;
  arr1[top] = n;
  return;
};

int dequeue()
{
    if (Front == - 1 || Front > Rear)
      return 0;
    else
    {
      Front = Front + 1;
      return arr[Front - 1];
    }
};

int pop()
{
  if(top == -1)
  {
    return 0;
  }
  else
  {
    top--;
    return arr1[top+1];
  }
};

void update(int y, int x) //DONE 
{
  //-1
  if (Wall[y][x]==-1){
    if (m[y-1][x]>m[y][x]+1)
    {
      m[y-1][x]=m[y][x]+1;
    }
    if (m[y+1][x]>m[y][x]+1)
    {
      m[y+1][x]=m[y][x]+1;
    }
    if (m[y][x+1]>m[y][x]+1)
    {
      m[y][x+1]=m[y][x]+1;
    }
  }

  //-2
  else if (Wall[y][x]==-2){
    if (m[y][x-1]>m[y][x]+1)
    {
      m[y][x-1]=m[y][x]+1;
    }
    if (m[y-1][x]>m[y][x]+1)
    {
      m[y-1][x]=m[y][x]+1;
    }
    if (m[y][x+1]>m[y][x]+1)
    {
      m[y][x+1]=m[y][x]+1;
    }
  }

  //-3
  else if (Wall[y][x]==-3){
    if (m[y-1][x]>m[y][x]+1)
    {
      m[y-1][x]=m[y][x]+1;
    }
    if (m[y+1][x]>m[y][x]+1)
    {
      m[y+1][x]=m[y][x]+1;
    }
    if (m[y][x-1]>m[y][x]+1)
    {
      m[y][x-1]=m[y][x]+1;
    }
  }

  //-4
  else if (Wall[y][x]==-4){
    if (m[y][x-1]>m[y][x]+1)
    {
      m[y][x-1]=m[y][x]+1;
    }
    if (m[y+1][x]>m[y][x]+1)
    {
      m[y+1][x]=m[y][x]+1;
    }
    if (m[y][x+1]>m[y][x]+1)
    {
      m[y][x+1]=m[y][x]+1;
    }
  }

  //-5
  else if (Wall[y][x]==-5){
    if (m[y+1][x]>m[y][x]+1)
    {
      m[y+1][x]=m[y][x]+1;
    }
    if (m[y][x+1]>m[y][x]+1)
    {
      m[y][x+1]=m[y][x]+1;
    }
  }

  //-6
  else if (Wall[y][x]==-6){
    if (m[y+1][x]>m[y][x]+1)
    {
      m[y+1][x]=m[y][x]+1;
    }
    if (m[y][x-1]>m[y][x]+1)
    {
      m[y][x-1]=m[y][x]+1;
    }
  }

  //-7
  else if (Wall[y][x]==-7){
    if (m[y-1][x]>m[y][x]+1)
    {
      m[y-1][x]=m[y][x]+1;
    }
    if (m[y][x-1]>m[y][x]+1)
    {
      m[y][x-1]=m[y][x]+1;
    }
  }

  //-8
  else if (Wall[y][x]==-8){
    if (m[y-1][x]>m[y][x]+1)
    {
      m[y-1][x]=m[y][x]+1;
    }
    if (m[y][x+1]>m[y][x]+1)
    {
      m[y][x+1]=m[y][x]+1;
    }
  }

  //-9
  else if (Wall[y][x]==-9){
    if (m[y+1][x]>m[y][x]+1)
    {
      m[y+1][x]=m[y][x]+1;
    }
    if (m[y-1][x]>m[y][x]+1)
    {
      m[y-1][x]=m[y][x]+1;
    }
  }

  //-10
  else if (Wall[y][x]==-10){
    if (m[y][x+1]>m[y][x]+1)
    {
      m[y][x+1]=m[y][x]+1;
    }
    if (m[y][x-1]>m[y][x]+1)
    {
      m[y][x-1]=m[y][x]+1;
    }
  }

  //-11
  else if (Wall[y][x]==-11){
    if (m[y+1][x]>m[y][x]+1)
    {
      m[y+1][x]=m[y][x]+1;
    }
  }

  //-12
  else if (Wall[y][x]==-12){
    if (m[y][x-1]>m[y][x]+1)
    {
      m[y][x-1]=m[y][x]+1;
    }
  }

  //-13
  else if (Wall[y][x]==-13){
    if (m[y-1][x]>m[y][x]+1)
    {
      m[y-1][x]=m[y][x]+1;
    }
  }

  //-14
  else if (Wall[y][x]==-14){
    if (m[y][x+1]>m[y][x]+1)
    {
      m[y][x+1]=m[y][x]+1;
    }
  }

  //-15
  else if (Wall[y][x]==-15){
    if (m[y+1][x]>m[y][x]+1)
    {
      m[y+1][x]=m[y][x]+1;
    }
    if (m[y-1][x]>m[y][x]+1)
    {
      m[y-1][x]=m[y][x]+1;
    }
    if (m[y][x+1]>m[y][x]+1)
    {
      m[y][x+1]=m[y][x]+1;
    }
    if (m[y][x-1]>m[y][x]+1)
    {
      m[y][x-1]=m[y][x]+1;
    }
  }
  else
  {
    if (Wall[y+1][x]!=-2 && Wall[y+1][x]!=-7 && Wall[y+1][x]!=-8 && Wall[y+1][x]!=-10 && Wall[y+1][x]!=-12 && Wall[y+1][x]!=-13 && Wall[y+1][x]!=-14 && Wall[y+1][x]!=9 )
    {
      if (m[y+1][x]>m[y][x]+1)
      {
        m[y+1][x]=m[y][x]+1;
      }
    }
    if (Wall[y-1][x]!=-4 && Wall[y-1][x]!=-5 && Wall[y-1][x]!=-6 && Wall[y-1][x]!=-10 && Wall[y-1][x]!=-12 && Wall[y-1][x]!=-11 && Wall[y-1][x]!=-14 && Wall[y-1][x]!=9 )
    {
      if (m[y-1][x]>m[y][x]+1)
      {
        m[y-1][x]=m[y][x]+1;
      }
    }
    if (Wall[y][x-1]!=-3 && Wall[y][x-1]!=-7 && Wall[y][x-1]!=-6 && Wall[y][x-1]!=-9 && Wall[y][x-1]!=-12 && Wall[y][x-1]!=-11 && Wall[y][x-1]!=-13 && Wall[y][x-1]!=9 )
    {
      if (m[y][x-1]>m[y][x]+1)
      {
        m[y][x-1]=m[y][x]+1;
      }
    }
    if (Wall[y][x+1]!=-1 && Wall[y][x+1]!=-5 && Wall[y][x+1]!=-8 && Wall[y][x+1]!=-9 && Wall[y][x+1]!=-14 && Wall[y][x+1]!=-11 && Wall[y][x+1]!=-13 && Wall[y][x+1]!=9 )
    {
      if (m[y][x+1]>m[y][x]+1)
      {
        m[y][x+1]=m[y][x]+1;
      }
    }

  }
  return;
}

void floodfill() //DONE
{

  for ( int i = 0 ; i <=8 ; i++){
    m[0 ][i] = -1;
    m[10][i] = -1;
  }

  for (int i = 0; i<=10;i++)
  {
    m[i][0] = -1;
    m[i][8] = -1;
  }
  for ( int i = 1 ; i<=9 ; i++)
  {
    for ( int j = 1 ; j <=7 ; j++)
    {
      m[i][j] = 64;
    }
  }
  m[5][4] = 0;


  
  for (int steps = 0; steps<= 63; steps++)
  {
    for (int j = 1; j<=9 ; j++)
    {
      for (int k = 1; k<=7 ; k++) 
      {
        if (m[j][k] == steps) {update(j,k);}
      }
    }
  }
  return;
};

void faceforward()
{
  while (angle()!=initial_angle)
  {
    digitalWrite(rwf, HIGH);
    digitalWrite(lwb, HIGH);
  }
  digitalWrite(rwf, LOW);
  digitalWrite(lwb, LOW);
}






void setup() {

  



  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);

  z = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
  initial_angle = z;

  Wire.write(0);
  Wire.endTransmission(true);

  pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(lwf, OUTPUT);
	pinMode(lwb, OUTPUT);
	pinMode(rwf, OUTPUT);
	pinMode(rwb, OUTPUT);
	
	// Turn off motors - Initial state
	digitalWrite(lwf, LOW);
	digitalWrite(lwb, LOW);
	digitalWrite(rwf, LOW);
	digitalWrite(rwb, LOW);
  digitalWrite(enA, HIGH);
  digitalWrite(enB, HIGH);

  pinMode(l_trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(l_echoPin, INPUT); // Sets the echoPin as an Input

  pinMode(f_trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(f_echoPin, INPUT); // Sets the echoPin as an Input

  pinMode(r_trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(r_echoPin, INPUT); // Sets the echoPin as an Input




  int lw=wall_check(l_trigPin, l_echoPin);
  int fw=wall_check(f_trigPin, f_echoPin);
  int rw=wall_check(r_trigPin, r_echoPin);
    if (lw==1 && fw==0 && rw==0)
  {
    Wall[8][0]= -5;
    forward();
    y--;
  }
  else if (lw==0 && fw==1 && rw==0)
  {
    Wall[8][0]= -10;
    right();
    or++; 
    forward();
    x++;
  }
  else if (lw==0 && fw==0 && rw==1)
  {
    Wall[8][0]= -6;
    forward();
    y--;
  }
  else if (lw==0 && fw==1 && rw==1)
  {
    Wall[8][0]= -12;

  }
  else if (lw==1 && fw==1 && rw==0)
  {
    Wall[8][0]= -14;
    right();
    or++; 
    forward();
    x++;
  }  
  else if (lw==1 && fw==0 && rw==1)
  {
    Wall[8][0]= -11;
    forward();
    y--;
  }
  else if (lw==0 && fw==0 && rw==0)
}


void loop() 
{
  int count = 0;
  while(count<3)
 {













  if(o==1)
  {
    switch(Wall[y][x]){
    case '-2':
      if(Wall[y][x+1]<0){
        if(Wall[y+1][x]<0){
          floodfill();
          if(m[y][x+1]>m[y+1][x]){
            right();
            o++;
            forward();
            y++;
          }
          else{
            forward();
            x++;
          }
        }
        else{
          right();
          o++;
          forward();
          y++;
        }
      }
      else{
        if(Wall[y+1][x]<0){
          forward();
          x++;
        }
        else{
          if(Wall[y][x+1]>Wall[y+1][x]){
            right();
            o++;
            forward();
            y++;
          }
          else{
            forward();
            x++;
          }
        }
      }
    case '-3':
      if(Wall[y-1][x]<0){
        if(Wall[y+1][x]<0){
          floodfill();
          if(m[y-1][x]>m[y+1][x]){
            right();
            o++;
            forward();
            y++;
          }
          else{
            left();
            o--;
            forward();
            y--;
          }
        }
        else{
          right();
          o++;
          forward();
          y++;
        }
      }
      else{
        if(Wall[y+1][x]<0){
          left();
          o--;
          forward();
          y--;
        }
        else{
          if(m[y-1][x]>m[y+1][x]){
            right();
            o++;
            forward();
            y++;
          }
          else{
            left();
            o--;
            forward();
            y--;
          }
        }
      }
    case '-4':
      if(Wall[y][x+1]<0){
        if(Wall[y-1][x]<0){
          floodfill();
          if(m[y][x+1]>m[y-1][x]){
            left();
            o--;
            forward();
            y--;
          }
          else{
            forward();
            x++;
          }
        }
        else{
          left();
          o--;
          forward();
          y--;
        }
      }
      else{
        if(Wall[y-1][x]<0){
          forward();
          x++;
        }
        else{
          if(m[y][x+1]>m[y-1][x]){
            left();
            o--;
            forward();
            y--;
          }
          else{
            forward();
            x++;
          }
        }
      }
    case '-6':
      left();
      o--;
      forward();
      y--;
    case '-7':
      right();
      o++;
      forward();
      y++;
    case '-10':
      forward();
      x++;
    case '-12':
      right();
      o++;
      right();
      o++;
      forward();
      x--;
    case '-15':
      if(Wall[y][x+1]<0){
        if(Wall[y-1][x]<0){
          floodfill();
          if(Wall[y+1][x]<0){
            if(m[y][x+1]>m[y-1][x]){
              if(m[y-1][x]>=m[y+1][x]){
                right();
                o++;
                forward();
                y++;
              }
              else{
                left();
                o--;
                forward();
                y--;
              }
            }
            else{
              if(m[y][x+1]<=m[y+1][x]){
                forward();
                x++;
              }
              else{
                right();
                o++;
                forward();
                y++;
              }
            }
          }
          else{
            right();
            o++;
            forward();
            y++;
          }
        }
        else{
          if(Wall[y+1][x]<0){
            left();
            o--;
            forward();
            y--;
          }
          else{
            if(m[y+1][x]>m[y-1][x]){
              left();
              o--;
              forward();
              y--;
            }
            else{
              right();
              o++;
              forward();
              y++;
            }
          }
        }
      }
      else{
        if(Wall[y-1][x]<0){
          if(Wall[y+1][x]<0){
            forward();
            x++;
          }
          else{
            if(m[y][x+1]>m[y+1][x]){
              right();
              o++;
              forward();
              y++;
            }
            else{
              forward();
              x++;
            }
          }
        }
        else{
          if(Wall[y+1][x]<0){
            if(m[y][x+1]>m[y-1][x]){
              left();
              o--;
              forward();
              y--;
            }
            else{
              forward();
              x++;
            }
          }
          else{
            if(m[y][x+1]>m[y-1][x]){
              if(m[y-1][x]>=m[y+1][x]){
                right();
                o++;
                forward();
                y++;
              }
              else{
                left();
                o--;
                forward();
                y--;
              }
            }
            else{
              if(m[y][x+1]<=m[y+1][x]){
                forward();
                x++;
              }
              else{
                right();
                o++;
                forward();
                y++;
              }
            }
          }
        }
      }
    }
  }

  else if(o==2)
  {
    switch(Wall[y][x]){
    case '-1':
      if(Wall[y][x+1]<0){
        if(Wall[y-1][x]<0){
          floodfill();
          if(m[y][x+1]<m[y-1][x]){
            left();
            o--;
            forward();
            x++;
          }
          else{
            forward();
            y++;
          }
        }
        else{
          forward();
          y++;
        }
      }
      else{
        if(Wall[y-1][x]<0){
          left();
          o--;
          forward();
          x++;
        }
        else{
          if(m[y][x+1]<m[y-1][x]){
            left();
            o--;
            forward();
            x++;
          }
          else{
            forward();
            y++;
          }
        }
      }
    case '-4':
      if(Wall[y][x-1]<0){
        if(Wall[y][x+1]<0){
          floodfill();
          if(m[y][x-1]>m[y][x+1]){
            left();
            o--;
            forward();
            x++;
          }
          else{
            right();
            o++;
            forward();
            x--;
          }
        }
        else{
          left();
          o--;
          forward();
          x++;
        }
      }
      else{
        if(Wall[y][x+1]<0){
          right();
          o++;
          forward();
          x--;
        }
        else{
          if(m[y][x-1]>m[y][x+1]){
            left();
            o--;
            forward();
            x++;
          }
          else{
            right();
            o++;
            forward();
            x--;
          }
        }
      }
    case '-3':
      if(Wall[y][x-1]<0){
        if(Wall[y+1][x]<0){
          floodfill();
          if(m[y][x-1]<m[y+1][x]){
            right();
            o++;
            forward();
            x--;
          }
          else{
            forward();
            y++;
          }
        }
        else{
          forward();
          y++;
        }
      }
      else{
        if(Wall[y+1][x]<0){
          right();
          o++;
          forward();
          x--;
        }
        else{
          if(m[y][x-1]<m[y+1][x]){
            right();
            o++;
            forward();
            x--;
          }
          else{
            forward();
            y++;
          }
        }
      }
    case '-5':
      left();
      o--;
      forward();
      x++;
    case '-6':
      right();
      o++;
      forward();
      x--;
    case '-9':
      forward();
      y++;
    case '-11':
      right();
      o++;
      right();
      o++;
      forward();
      y--;
    case '-15':
      if(Wall[y+1][x]<0){
        if(Wall[y][x+1]<0){
          floodfill();
          if(Wall[y][x-1]<0){
            if(m[y+1][x]>m[y][x+1]){
              if(m[y][x+1]>=m[y][x-1]){
                right();
                o++;
                forward();
                x--;
              }
              else{
                left();
                o--;
                forward();
                x++;
              }
            }
            else{
              if(m[y+1][x]<=m[y][x-1]){
                forward();
                y++;
              }
              else{
                right();
                o++;
                forward();
                x--;
              }
            }
          }
          else{
            right();
            o++;
            forward();
            x--;
          }
        }
        else{
          if(Wall[y][x-1]<0){
            left();
            o--;
            forward();
            x++;
          }
          else{
            if(m[y][x-1]>m[y][x+1]){
              left();
              o--;
              forward();
              x++;
            }
            else{
              right();
              o++;
              forward();
              x--;
            }
          }
        }
      }
      else{
        if(Wall[y][x+1]<0){
          if(Wall[y][x-1]<0){
            forward();
            y++;
          }
          else{
            if(m[y+1][x]>m[y][x-1]){
              right();
              o++;
              forward();
              x--;
            }
            else{
              forward();
              y++;
            }
          }
        }
        else{
          if(Wall[y][x-1]<0){
            if(m[y+1][x]>m[y][x+1]){
              left();
              o--;
              forward();
              x++;
            }
            else{
              forward();
              y++;
            }
          }
          else{
            if(m[y+1][x]>m[y][x+1]){
              if(m[y][x+1]>=m[y][x-1]){
                right();
                o++;
                forward();
                x--;
              }
              else{
                left();
                o--;
                forward();
                x++;
              }
            }
            else{
              if(m[y+1][x]<=m[y][x-1]){
                forward();
                y++;
              }
              else{
                right();
                o++;
                forward();
                x--;
              }
            }
          }
        }
      }
    }
  }

  else if(o==3)
  {
    switch(Wall[y][x]){
    case '-4':
      if(Wall[y][x-1]<0){
        if(Wall[y-1][x]<0){
          floodfill();
          if(m[y][x-1]>m[y-1][x]){
            right();
            o++;
            forward();
            y--;
          }
          else{
            forward();
            x--;
          }
        }
        else{
          right();
          o++;
          forward();
          y--;
        }
      }
      else{
        if(Wall[y-1][x]<0){
          forward();
          x--;
        }
        else{
          if(m[y][x-1]>m[y-1][x]){
            right();
            o++;
            forward();
            y--;
          }
          else{
            forward();
            x--;
          }
        }
      }
    case '-1':
      if(Wall[y+1][x]<0){
        if(Wall[y-1][x]<0){
          floodfill();
          if(m[y+1][x]>=m[y-1][x]){
            right();
            o++;
            forward();
            y--;
          }
          else{
            left();
            o--;
            forward();
            y++;
          }
        }
        else{
          right();
          o++;
          forward();
          y--;
        }
      }
      else{
        if(Wall[y-1][x]<0){
          left();
          o--;
          forward();
          y++;
        }
        else{
          if(m[y+1][x]>=m[y-1][x]){
            right();
            o++;
            forward();
            y--;
          }
          else{
            left();
            o--;
            forward();
            y++;
          }
        }
      }
    case '-2':
      if(Wall[y][x-1]<0){
        if(Wall[y+1][x]<0){
          floodfill();
          if(m[y][x-1]>m[y+1][x]){
            left();
            o--;
            forward();
            y++;
          }
          else{
            forward();
            x--;
          }
        }
        else{
          left();
          o--;
          forward();
          y++;
        }
      }
      else{
        if(Wall[y+1][x]<0){
          forward();
          x--;
        }
        else{
          if(m[y][x-1]>m[y+1][x]){
            left();
            o--;
            forward();
            y++;
          }
          else{
            forward();
            x--;
          }
        }
      }
    case '-8':
      left();
      o--;
      forward();
      y++;
    case '-5':
      right();
      o++;
      forward();
      y--;
    case '-10':
      forward();
      x--;
    case '-14':
      right();
      o++;
      right();
      o++;
      forward();
      x++;
    case '-15':
      if(Wall[y][x-1]<0){
        if(Wall[y+1][x]<0){
          floodfill();
          if(Wall[y-1][x]<0){
            if(m[y][x-1]>m[y+1][x]){
              if(m[y+1][x]>=m[y-1][x]){
                right();
                o++;
                forward();
                y--;
              }
              else{
                left();
                o--;
                forward();
                y++;
              }
            }
            else{
              if(m[y][x-1]<=m[y-1][x]){
                forward();
                x--;
              }
              else{
                right();
                o++;
                forward();
                y--;
              }
            }
          }
          else{
            right();
            o++;
            forward();
            y--;
          }
        }
        else{
          if(Wall[y-1][x]<0){
            left();
            o--;
            forward();
            y++;
          }
          else{
            if(m[y-1][x]>m[y+1][x]){
              left();
              o--;
              forward();
              y++;
            }
            else{
              right();
              o++;
              forward();
              y--;
            }
          }
        }
      }
      else{
        if(Wall[y+1][x]<0){
          if(Wall[y-1][x]<0){
            forward();
            x--;
          }
          else{
            if(m[y][x-1]>m[y-1][x]){
              right();
              o++;
              forward();
              y--;
            }
            else{
              forward();
              x--;
            }
          }
        }
        else{
          if(Wall[y-1][x]<0){
            if(m[y][x-1]>m[y+1][x]){
              left();
              o--;
              forward();
              y++;
            }
            else{
              forward();
              x--;
            }
          }
          else{
            if(m[y][x-1]>m[y+1][x]){
              if(m[y+1][x]>=m[y-1][x]){
                right();
                o++;
                forward();
                y--;
              }
              else{
                left();
                o--;
                forward();
                y++;
              }
            }
            else{
              if(m[y][x-1]<=m[y-1][x]){
                forward();
                x--;
              }
              else{
                right();
                o++;
                forward();
                y--;
              }
            }
          }
        }
      }
    }
  }

  else
  {
    switch(Wall[y][x]){
    case '-3':
      if(Wall[y][x-1]<0){
        if(Wall[y+1][x]<0){
          floodfill();
          if(m[y][x-1]<m[y+1][x]){
            left();
            o--;
            forward();
            x--;
          }
          else{
            forward();
            y--;
          }
        }
        else{
          forward();
          y--;
        }
      }
      else{
        if(Wall[y+1][x]<0){
          left();
          o--;
          forward();
          x--;
        }
        else{
          if(m[y][x-1]<m[y+1][x]){
            left();
            o--;
            forward();
            x--;
          }
          else{
            forward();
            y--;
          }
        }
      }
    case '-2':
      if(Wall[y][x+1]<0){
        if(Wall[y][x=1]<0){
          floodfill();
          if(m[y][x+1]>m[y][x-1]){
            left();
            o--;
            forward();
            x--;
          }
          else{
            right();
            o++;
            forward();
            x++;
          }
        }
        else{
          left();
          o--;
          forward();
          x--;
        }
      }
      else{
        if(Wall[y][x-1]<0){
          right();
          o++;
          forward();
          x++;
        }
        else{
          if(m[y][x+1]>m[y][x-1]){
            left();
            o--;
            forward();
            x--;
          }
          else{
            right();
            o++;
            forward();
            x++;
          }
        }
      }
    case '-1':
      if(Wall[y][x+1]<0){
        if(Wall[y-1][x]<0){
          floodfill();
          if(m[y][x+1]<m[y-1][x]){
            right();
            o++;
            forward();
            x++;
          }
          else{
            forward();
            y--;
          }
        }
        else{
          forward();
          y--;
        }
      }
      else{
        if(Wall[y-1][x]<0){
          right();
          o++;
          forward();
          x++;
        }
        else{
          if(m[y][x+1]<m[y-1][x]){
            right();
            o++;
            forward();
            x++;
          }
          else{
            forward();
            y--;
          }
        }
      }
    case '-7':
      left();
      o--;
      forward();
      x--;
    case '-8':
      right();
      o++;
      forward();
      x++;
    case '-9':
      forward();
      y--;
    case '-13':
      right();
      o++;
      right();
      o++;
      forward();
      y++;
    case '-15':
      if(Wall[y-1][x]<0){
        if(Wall[y][x-1]<0){
          floodfill();
          if(Wall[y][x+1]<0){
            if(m[y-1][x]>m[y][x-1]){
              if(m[y][x-1]>=m[y][x+1]){
                right();
                o++;
                forward();
                x++;
              }
              else{
                left();
                o--;
                forward();
                x--;
              }
            }
            else{
              if(m[y-1][x]<=m[y][x+1]){
                forward();
                y--;
              }
              else{
                right();
                o++;
                forward();
                x++;
              }
            }
          }
          else{
            right();
            o++;
            forward();
            x++;
          }
        }
        else{
          if(Wall[y][x+1]<0){
            left();
            o--;
            forward();
            x--;
          }
          else{
            if(m[y][x+1]>m[y][x-1]){
              left();
              o--;
              forward();
              x--;
            }
            else{
              right();
              o++;
              forward();
              x++;
            }
          }
        }
      }
      else{
        if(Wall[y][x-1]<0){
          if(Wall[y][x+1]<0){
            forward();
            y--;
          }
          else{
            if(m[y-1][x]>m[y][x+1]){
              right();
              o++;
              forward();
              x++;
            }
            else{
              forward();
              y--;
            }
          }
        }
        else{
          if(Wall[y][x+1]<0){
            if(m[y-1][x]>m[y][x-1]){
              left();
              o--;
              forward();
              x--;
            }
            else{
              forward();
              y--;
            }
          }
          else{
            if(m[y-1][x]>m[y][x-1]){
              if(m[y][x-1]>=m[y][x+1]){
                right();
                o++;
                forward();
                x++;
              }
              else{
                left();
                o--;
                forward();
                x--;
              }
            }
            else{
              if(m[y-1][x]<=m[y][x+1]){
                forward();
                y--;
              }
              else{
                right();
                o++;
                forward();
                x++;
              }
            }
          }
        }
      }
    }
  }
  




  config();
  if(x==4 && y==5) //fjgodg
    count++;  
 } 
 floodfill();

  o=0;

  while(x!=4 || y!=5) //gsdfgfdghsf
  {
  if(o==0){
  switch(Wall[y][x]){
  case '-1':
    if(m[y-1][ x]==m[y][x]-1){
      Rear++;
      enqueue(1);
      push(1);
      y--;
    }
      
    else{
      Rear++;
      enqueue(2);
      push(2);
      o++;
      x++;
    }

  case '-2':
    if(m[y][x+1]==m[y][x]-1){

    Rear++;
    enqueue(2);
    push(2);
    o++;
    x++; 
    }   
        
    else{
    Rear++;
    enqueue(3);
    push(3);
    o--;
    x--;
    }

  case '-3':
    if(m[y-1][ x]==m[y][x]-1){
      Rear++;
      enqueue(1);
      push(1);
      y--;
    }
      
    else{
      Rear++;
      enqueue(3);
      push(3);
      o--;
      x--;
    }

  case '-7':
    Rear++;
    enqueue(3);
    push(3);
    o--;
    x--;
  
  case '-8':
    Rear++;
    enqueue(2);
    push(2);
    o++;
    x++;

  case '-9':
    Rear++;
    enqueue(1);
    push(1);
    y--;
 
  case '-15':
    if(m[y-1][x]==m[y][x]-1){
      Rear++;
      enqueue(1);
      push(1);
      y--;
    }

    else if(m[y][x+1]==m[y][x]-1){
      Rear++;    
      enqueue(2);
      push(2);
      o++;
      x++;
    }
      
    else{
      Rear++;
      enqueue(3);
      push(3);
      o--;
      x--;
    }

  }
  }

  if(o==1){
  switch(Wall[y][x]){
  case '-2':
    if(m[y][x+1]==m[y][x]-1){
      Rear++;
      enqueue(1);
      push(1);
      x++;
    }

    else{
      Rear++;
      enqueue(2);
      push(2);
      o++;
      y++;
    }

  case '-3':
    if(m[y+1][x]==m[y][x]-1){
      Rear++;
      enqueue(2);
      push(2);
      o++;
      y++;
    }

    else{
      Rear++;
      enqueue(3);
      push(3);
      o--;
      y--;
    }

  case '-4':
    if(m[y][x+1]==m[y][x]-1){
      Rear++;
      enqueue(1);
      push(1);
      x++;
    }

    else{
      Rear++;
      enqueue(3);
      push(3);
      o--;
      y--;
    }
  case '-6':
    Rear++;
    enqueue(3);
    push(3);
    o--;
    y--;

  case '-7':
    Rear++;
    enqueue(2);
    push(2);
    o++;
    y++;

  case '-10':
    Rear++;
    enqueue(1);
    push(1);
    x++;
  
  case '-15':
    if(m[y][x+1]==m[y][x]-1){
      Rear++;
      enqueue(1);
      push(1);
      x++;
    }

    else if(m[y+1][x]==m[y][x]-1){
      Rear++;
      enqueue(2);
      push(2);
      o++;
      y++;
    }
      
    else{
      Rear++;
      enqueue(3);
      push(3);
      o--;
      y--;
    }

  }
  }

  if(o==3){
  case
    if(m[y][x-1]==m[y][x]-1){
      Rear++;
      enqueue(1);
      push(1);
      x--;
    }

    else{
      Rear++;
      enqueue(2);
      push(2);
      o++;
      y--;
    }

  case '-1':
    if(m[y-1][x]==m[y][x]-1){
      Rear++;
      enqueue(2);
      push(2);
      o++;
      y--;
    }

    else{
      Rear++;
      enqueue(3);
      push(3);
      o--;
      y++;
    }

  case '-2':
    if(m[y][x-1]==m[y][x]-1){
      Rear++;
      enqueue(1);
      push(1);
      x--;
    }

    else{
      Rear++;
      enqueue(3);
      push(3);
      o--;
      y++;
    }
  case '-8':
    Rear++;
    enqueue(3);
    push(3);
    o--;
    y++;

  case '-5':
    Rear++;
    enqueue(2);
    push(2);
    o++;
    y--;

  case '-10':
    Rear++;
    enqueue(1);
    push(1);
    x--;
  
  case '-15':
    if(m[y][x-1]==m[y][x]-1){
      Rear++;
      enqueue(1);
      push(1);
      x--;
    }

    else if(m[y-1][x]==m[y][x]-1){
      Rear++;
      enqueue(2);
      push(2);
      o++;
      y--;
    }
      
    else{
      Rear++;
      enqueue(3);
      push(3);
      o--;
      y++;
    }

  }
  }

  if(o==2){
  switch(Wall[y][x]){
  case '-3':
    if(m[y+1][ x]==m[y][x]-1){
      Rear++;
      enqueue(1);
      push(1);
      y++;
    }
      
    else{
      Rear++;
      enqueue(2);
      push(2);
      o++;
      x--;
    }

  case '-4':
    if(m[y][x-1]==m[y][x]-1){
    Rear++;
    enqueue(2);
    push(2);
    o++;
    x--; 
    }   
        
    else{
    Rear++;
    enqueue(3);
    push(3);
    o--;
    x++;
    }

  case '-1':
    if(m[y+1][ x]==m[y][x]-1){
      Rear++;
      enqueue(1);
      push(1);
      y++;
    }
      
    else{
      Rear++;
      enqueue(3);
      push(3);
      o--;
      x++;
    }

  case '-5':
    Rear++;
    enqueue(3);
    push(3);
    o--;
    x++;
  
  case '-6':
    Rear++;
    enqueue(2);
    push(2);
    o++;
    x--;

  case '-9':
    Rear++;
    enqueue(1);
    push(1);
    y++;
 
  case '-15':
    if(m[y+1][x]==m[y][x]-1){
      Rear++;
      enqueue(1);
      push(1);
      y++;
    }

    else if(m[y][x-1]==m[y][x]-1){
      Rear++;
      enqueue(2);
      push(2);
      o++;
      x--;
    }
      
    else{
      Rear++;
      enqueue(3);
      push(3);
      o--;
      x++;
    }

  }
  }
  o=o%4;
  }
  
  switch(o)
  {
    case '0':
      faceforward();
      right();
      right();
    case '1':
      faceforward();
      left();
    case '2':
      faceforward();
    case '3':
      faceforward();
      right();      
  }

  while(x!=0){
  x=pop();
  switch(x)
  {
    case '1':
      forward();
    case '3':
      right();
      forward();
    case '2':
      left();
      forward();
  }
  }

faceforward();

  while(x!=0){
  x=dequeue();
  switch(x)
  {
    case '1':
      forward();
    case '2':
      right();
      forward();
    case '3':
      left();
      forward();
  }
  }
  
  

}
