#define left_motor_positive 8
#define left_motor_negative 7
#define right_motor_positive 2
#define right_motor_negative 4

#define en1 6 //left
#define en2 3  

#define button 12

void forward(int,int);
void left(int,int);
void right(int,int);
void sharpturn(int,int);
void stopmotor();
void leds();
void reverse();
void pid();
void calibrate();
void linefollow();

int P, D, I=0, previousError=0, PIDvalue, error;
int lsp, rsp;
int lfspeed = 70;

int Status=0;
int led=9;
int reading;
float Kp = 0;
float Kd = 0;
float Ki = 0 ;

void dryrun();
void dryactual();
void actualrun();
void recIntersection(char);
char path[100] = "";
unsigned char pathLength = 0; // the length of the path
int pathIndex = 0;
void mazeTurn (char dir);


int minValues[6], maxValues[6], threshold[6];

void setup()
{
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(led, OUTPUT);
  pinMode(left_motor_positive,OUTPUT);
  pinMode(left_motor_negative,OUTPUT);
  pinMode(right_motor_positive,OUTPUT);
  pinMode(right_motor_negative,OUTPUT);
  pinMode(en1,OUTPUT);
  pinMode(en2,OUTPUT);
}


void loop()
{
  reading=digitalRead(button);
  if(reading==1 && Status==0)
  {
  delay(350);
  calibrate();
  stopmotor();
  Status++;
  }
  else if(reading==1 && Status==1)
    {
      dryrun();
    }   
   else if(reading==1 && Status==2)
 {
  dryractual();
//  actualrun();
  Status = 2;
//  pathIndex = 0;
 }
}
void dryrun()
{
  while (Status==1)
{
       if(analogRead(5) > threshold[5] && analogRead(4) > threshold[4]&& analogRead(1) > threshold[1] && analogRead(0) > threshold[0] && digitalRead(10)==1 && digitalRead(11)==1) 
    {
      delay(88);
      if(analogRead(5) > threshold[5] && analogRead(4) > threshold[4]&& analogRead(1) > threshold[1] && analogRead(0) > threshold[0] && digitalRead(10)==1 && digitalRead(11)==1)
      {
        stopmotor();
        leds();
        Status++;
      }
      else{
        //recIntersection('L');
        left(55,55);
        
      }
        
      
    }
       else if(analogRead(5)<threshold[5]&&analogRead(4)<threshold[4]&&analogRead(3)<threshold[3]&&analogRead(2)<threshold[2]&&analogRead(1)<threshold[1]&&analogRead(0)<threshold[0])
    {
      // recIntersection('B');
      sharpturn(55,55);
      
      
    }
    else if (analogRead(5) > threshold[5] && analogRead(4) > threshold[4])
    {
      //recIntersection('L');
      left(55,55);
     
    }

    else if (analogRead(0) > threshold[0] && analogRead(1) > threshold[1])
    { 
      for(int i=0;i<10;i++)
      {
        if(analogRead(5) > threshold[5] && analogRead(4) > threshold[4])
        {
        //recIntersection('L');
          left(55,55);
          
          goto bot; 
        }
      }
      //recIntersection('L');
      right(55,55);
      bot:
      Serial.println("");
    }
    else if ((analogRead(2) + analogRead(3)) > (threshold[2] + threshold[3] ))
    {
      pid();
    }
  }
}
void  actualrun()
{
  while(Status==2)
  {
     if(analogRead(5) > threshold[5] && analogRead(4) > threshold[4]&& analogRead(1) > threshold[1] && analogRead(0) > threshold[0] && digitalRead(10)==1 && digitalRead(11)==1) 
    {
      delay(88);
      if(analogRead(5) > threshold[5] && analogRead(4) > threshold[4]&& analogRead(1) > threshold[1] && analogRead(0) > threshold[0] && digitalRead(10)==1 && digitalRead(11)==1)
      {
        stopmotor();
        leds();
        Status++;
      }
      else{
        if (pathIndex >= pathLength)
        {
          stopmotor();
        leds();
        Status++;
        }
        
      else
      {
        mazeTurn (path[pathIndex]);
        pathIndex++;
      }
      }
        
      
    }
       else if(analogRead(5)<threshold[5]&&analogRead(4)<threshold[4]&&analogRead(3)<threshold[3]&&analogRead(2)<threshold[2]&&analogRead(1)<threshold[1]&&analogRead(0)<threshold[0])
    {
       if (pathIndex >= pathLength)
       {
        stopmotor();
        leds();
        Status++;
      }
      else
      {
        mazeTurn (path[pathIndex]);
        pathIndex++;
      }
      
    }
    else if (analogRead(5) > threshold[5] && analogRead(4) > threshold[4])
    {
      if (pathIndex >= pathLength)
      {
        stopmotor();
        leds();
        Status++;
      }
        
      else
      {
        mazeTurn (path[pathIndex]);
        pathIndex++;
      }
    }

    else if (analogRead(0) > threshold[0] && analogRead(1) > threshold[1])
    { 
      for(int i=0;i<10;i++)
      {
        if(analogRead(5) > threshold[5] && analogRead(4) > threshold[4])
        {
           if (pathIndex >= pathLength)
      {
        stopmotor();
        leds();
        Status++;
      }
        
      else
      {
        mazeTurn (path[pathIndex]);
        pathIndex++;
      }
          goto bot; 
        }
      }
      if (pathIndex >= pathLength)
      {
        stopmotor();
        leds();
        Status++;
      }
       
      else
      {
        mazeTurn (path[pathIndex]);
        pathIndex++;
      }
      bot:
      Serial.println("");
    }
    else if ((analogRead(2) + analogRead(3)) > (threshold[2] + threshold[3] ))
    {
      pid();
    }
  }
}
void pid()
{
  Kp = 0.0005* (800-((analogRead(2)+analogRead(3)))/2);
      Kd =50 * Kp; //40
      //Ki = 0.0001;
      linefollow();
}
void linefollow()
{
  int error = (analogRead(4) - analogRead(1));

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 255) {
    lsp = 90;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 90;
  }
  if (rsp < 0) {
    rsp = 0;
  }
 forward(lsp,rsp);

}

void calibrate()
{
  
  for ( int i = 0; i < 6; i++)
  {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }
  
  for (int i = 0; i < 3000; i++)
  {
   sharpturn(55,55);
    

    for ( int i = 0; i < 6; i++)
    {
      if (analogRead(i) < minValues[i])
      {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i])
      {
        maxValues[i] = analogRead(i);
      }
    }
    
  }

  for ( int i = 0; i < 6; i++)
  {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  Serial.println();
  
 stopmotor();
}
void recIntersection(char Direction)
{
  path[pathLength] = Direction; // Store the intersection in the path variable.
  pathLength ++;
  simplifyPath(); // Simplify the learned path.
}

void simplifyPath()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if (pathLength < 3 || path[pathLength - 2] != 'B')
    return;

  int totalAngle = 0;
  int i;
  for (i = 1; i <= 3; i++)
  {
    switch (path[pathLength - i])
    {
      case 'R':
        totalAngle += 90;
        break;
      case 'L':
        totalAngle += 270;
        break;
      case 'B':
        totalAngle += 180;
        break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  totalAngle = totalAngle % 360;

  // Replace all of those turns with a single one.
  switch (totalAngle)
  {
    case 0:
      path[pathLength - 3] = 'S';
      break;
    case 90:
      path[pathLength - 3] = 'R';
      break;
    case 180:
      path[pathLength - 3] = 'B';
      break;
    case 270:
      path[pathLength - 3] = 'L';
      break;
  }
  // The path is now two steps shorter.
  pathLength -= 2;
}

void mazeTurn (char dir)
{
  
  switch (dir)
  {
    case 'L': // Turn Left
      left(55,55);
      break;

    case 'R': // Turn Right
      right(55,55);
      break;

    case 'B': // Turn Back
      sharpturn(55,55);
      break;

    case 'S': // Go Straight
      pid();
      break;
  }
}
void forward(int spd1, int spd2)
{

  analogWrite(en1, spd1);
  analogWrite(en2, spd2);
  digitalWrite(left_motor_positive, HIGH);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, HIGH);
  digitalWrite(right_motor_negative, LOW);
}

void right(int spd1,int spd2)
{
  analogWrite(en1, spd1);
  analogWrite(en2, spd2);
  digitalWrite(left_motor_positive, HIGH);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, HIGH);
}

void left(int spd1,int spd2)
{
  analogWrite(en1, spd1);
  analogWrite(en2, spd2);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, HIGH);
  digitalWrite(right_motor_positive, HIGH);
  digitalWrite(right_motor_negative, LOW);
}
void sharpturn(int spd1,int spd2)
{
  analogWrite(en1, spd1);
  analogWrite(en2, spd2);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, HIGH);
  digitalWrite(right_motor_positive, HIGH);
  digitalWrite(right_motor_negative, LOW);
}
void stopmotor()
{
  
  analogWrite(en1, 0);
  analogWrite(en2, 0);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, LOW);
}
void leds()
{
  digitalWrite(led,HIGH);
  delay(2000);
  digitalWrite(led,LOW);
}

void dryractual(){
  while (Status==2)
{
       if(analogRead(5) > threshold[5] && analogRead(4) > threshold[4]&& analogRead(1) > threshold[1] && analogRead(0) > threshold[0] && digitalRead(10)==1 && digitalRead(11)==1) 
    {
      delay(88);
      if(analogRead(5) > threshold[5] && analogRead(4) > threshold[4]&& analogRead(1) > threshold[1] && analogRead(0) > threshold[0] && digitalRead(10)==1 && digitalRead(11)==1)
      {
        stopmotor();
        leds();
        Status++;
      }
      else{
        //recIntersection('L');
        left(55,55);
        
      }
        
      
    }
       else if(analogRead(5)<threshold[5]&&analogRead(4)<threshold[4]&&analogRead(3)<threshold[3]&&analogRead(2)<threshold[2]&&analogRead(1)<threshold[1]&&analogRead(0)<threshold[0])
    {
      // recIntersection('B');
      sharpturn(55,55);
      
      
    }
    else if (analogRead(5) > threshold[5] && analogRead(4) > threshold[4])
    {
      //recIntersection('L');
      left(55,55);
     
    }

    else if (analogRead(0) > threshold[0] && analogRead(1) > threshold[1])
    { 
      for(int i=0;i<10;i++)
      {
        if(analogRead(5) > threshold[5] && analogRead(4) > threshold[4])
        {
        //recIntersection('L');
          left(55,55);
          
          goto bot; 
        }
      }
      //recIntersection('L');
      right(55,55);
      bot:
      Serial.println("");
    }
    else if ((analogRead(2) + analogRead(3)) > (threshold[2] + threshold[3] ))
    {
      pid();
    }
  }
}
