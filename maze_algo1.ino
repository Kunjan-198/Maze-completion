int path=0;     //path=0 -> left line follower , //path=1 -> shortest path follower

int count=0;    //starting value for keeping robot stop
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int n=1000;     //delay time to check intersection point
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int i=0;        //starting value for keeping robot wait to cross junction  
int duration;
int disti;
int threshold=5; //threshold value

int start_button=19;//pin to start shortest path
//ultrasonic trigger and echo pin
int ult_trigger[]={13,2,0};
int ult_echo[]={12,15,4};

//distance measured by ultrasonic
int ult_distance[]={0,0,0};

//value stored for effecient path (1=left, 2=straight, 3=right, 4=back) 
int string[30];
int k=0;            //pointer to short_path generate
int w=0;            //pointer to short_path running 

int motor_left1=16;
int motor_left2=17;
int motor_right1=5;
int motor_right2=18;
//int en1=1;
//int en2=2;

int LED1=14;
int LED2=27;
int LED3=26;

void setup(){
  Serial.begin(9600); // Starts the serial communication
  pinMode(ult_trigger[0], OUTPUT); 
  pinMode(ult_echo[0], INPUT); 
  pinMode(ult_trigger[1], OUTPUT); 
  pinMode(ult_echo[1], INPUT);
  pinMode(ult_trigger[2], OUTPUT);
  pinMode(ult_echo[2], INPUT);
  pinMode(motor_left1, OUTPUT);
  pinMode(motor_left2, OUTPUT);
  pinMode(motor_right1, OUTPUT);
  pinMode(motor_right2, OUTPUT);
  pinMode(start_button, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
//  pinMode(en1, OUTPUT);
//  pinMode(en2, OUTPUT);
}

void left_turn(){
  analogWrite(motor_left1,0);
  analogWrite(motor_left2,150);
  analogWrite(motor_right1,150);
  analogWrite(motor_right2,0);
}

void right_turn(){
  analogWrite(motor_left1,150);
  analogWrite(motor_left2,0);
  analogWrite(motor_right1,0);
  analogWrite(motor_right2,150);
}

void straight(){
  analogWrite(motor_left1,0);
  analogWrite(motor_left2,150);
  analogWrite(motor_right1,0);
  analogWrite(motor_right2,150);
}

void stop_robot(){
  analogWrite(motor_left1,0);
  analogWrite(motor_left2,0);
  analogWrite(motor_right1,0);
  analogWrite(motor_right2,0);
}

int distance(int trig, int echo){
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  disti = duration * 0.034/2;
  if (disti>threshold){
    disti=0;
  }else{
    disti=1;  
  }
  return disti;
}

void left_wall_follower(){

  if(ult_distance[0]==0){
    left_turn();
  }else if(ult_distance[1]==0){
    straight();
  }else if(ult_distance[2]==0){
    right_turn();
  }

  if(ult_distance[1]+(ult_distance[0]*(1-ult_distance[2]))+(ult_distance[2]*(1-ult_distance[0]))){
    i++;
    if(i==n){
      i=0;
    }
    if(i==0){
        if(ult_distance[0]==0){
        string[k++]=1; 
        }else if(ult_distance[1]==0){
        string[k++]=2; 
        }else if(ult_distance[2]==0){
        string[k++]=3; 
        }else{
        string[k++]=4;
        }  
    }
  }
}

void minimizing_str(){
  int x=1;
  int str_len=k;
  while(x==1){
    x=0;
    for(int y=0; y<str_len;y++){
      if(string[y]==1 and string[y+1]==4 and string[y+2]==1){
        string[y]=0;
        string[y+1]=2;
        string[y+2]=0;
        x=1;
      }
      else if(string[y]==1 and string[y+1]==4 and string[y+2]==3){
        string[y]=0;
        string[y+1]=4;
        string[y+2]=0;
        x=1;
      }
      else if(string[y]==1 and string[y+1]==4 and string[y+2]==2){
        string[y]=0;
        string[y+1]=3;
        string[y+2]=0;
        x=1;
      }
      else if(string[y]==3 and string[y+1]==4 and string[y+2]==1){
        string[y]=0;
        string[y+1]=4;
        string[y+2]=0;
        x=1;
      }
      else if(string[y]==2 and string[y+1]==4 and string[y+2]==1){
        string[y]=0;
        string[y+1]=3;
        string[y+2]=0;
        x=1;
      }
      else if(string[y]==2 and string[y+1]==4 and string[y+2]==4){
        string[y]=0;
        string[y+1]=2;
        string[y+2]=0;
        x=1;
      }
      if(x==1){y=y+2;}
    }
    int p=0,q=0;
    while(q!=str_len){
      if(string[q]!=0){
        string[p++]=string[q++];
      }else{
        q++;  
      }
    }
    str_len=p;
  }
}

void shortest_path(){
   if(ult_distance[1]+(ult_distance[0]*(1-ult_distance[2]))+(ult_distance[2]*(1-ult_distance[0]))){
      if(string[w]==1){
        int t=0;
        while(t<n){
          left_turn();
          t++;
        }
        t=0;
        while(t<n/2){
          straight();
          t++;
        }
        w++;
        t=0;
      }else if(string[w]==3){
        int t=0;
        while(t<n){
          right_turn();
          t++;
        }
        t=0;
        while(t<n/2){
          straight();
          t++;
        }
        w++;
        t=0;
      }else if(string[w]==2){
        int t=0;
        while(t<n/2){
          straight();
          t++;
        }
        w++;
        t=0;   
      }
   }
   else{
      straight(); 
   }  
}

void loop(){
  
  ult_distance[0]=distance(ult_trigger[0],ult_echo[0]);
  ult_distance[1]=distance(ult_trigger[1],ult_echo[1]);
  ult_distance[2]=distance(ult_trigger[2],ult_echo[2]);

  if(ult_distance[0]==0 and ult_distance[1]==0 and ult_distance[2]==0){
    count=0;
    while(count<n){
      stop_robot();
      w=0;
    }
    if(path==0){
    minimizing_str();
    path=1;
    }
  }
  if(path==0){
    left_wall_follower();  
  }else if(path==1 and start_button==1){
    shortest_path();  
  }
  
}
