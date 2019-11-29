//for atan2(y,x)
#include <math.h> 

//for servo control
#include <Servo.h>
//declare my three servos (pins are declared later)
Servo Ser_1;
Servo Ser_2;
Servo Ser_3;

//if you use a led for error indicator:
int led=5, led_state=0;

//all variables
//change l1, l2 and l3 according to your robotic arm lengths.
double l1=8, l2=7.3, l3=10.6; //the lengths of the arms
int x, y, z; //for the position in space
double Th1=90, Th2=90, Th3=90; //initial angles (when arduino starts)
double Final_Deg1, Final_Deg2, Final_Deg3, UDiff1, UDiff2, UDiff3, Diff1, Diff2, Diff3, C3_Result;
bool Th3_Ph_Range_Error=false, Th2_Ph_Range_Error=false, C3_Error=false; 
int i; //our helping hand
double Th1_Calc_1, Th2_Calc_1, Th3_Calc_1; //used for first solution
double Th1_Calc_2, Th2_Calc_2, Th3_Calc_2; //used for second solution

//below, the functions that convert from the practical values to the theoritical values, change them accoridingly.
double TH1(int Deg){
  return Deg;
}

double TH2(int Deg){
  return (-Deg) + 150; 
}

double TH3(int Deg){
  return Deg +125;
}

//Angle limits, change accordingly
float Th2_max_negative= -47;
float Th2_max_positive= 67;
float Th3_max_positive= +153;
float Th3_max_negative= -135.5;

//change the speed (lower number, => faster speed and vise versa)
int delayTime=25;



//all functions

//function used many times to convert from rad to deg
double Rad_To_Deg(double Conv){
  return Conv* 57.2957;
}

//will be needed for the rest of functions
double C3(int x,int y,int z)
{
  C3_Result = (sq(x)+sq(y)+sq(z-l1)-sq(l2)-sq(l3))/(2*l2*l3);
  
  //C3 has limitation due to square root. these limitations should not be exceeded (dont enter big x,y,z)
  if(C3_Result<(-1) || C3_Result>1)
  {
    Serial.println("out of range, C3_Error");
    C3_Error = true;
  }
  else{
    C3_Error=false;
    return C3_Result;
  }
}

//will be needed for the rest of functions. for Up elbow solution, let last parameter be false by default. For down elbow solution, S3 should be <0 so let change last parameter to true.
//the limitations of the sqrt have already been solved to the C3_Result, so there is no need to check the S3 sqrt limitation.
double S3(int x,int y,int z, bool Second_Solution=false)
{
  if (Second_Solution==false){//up elbow slution
      return sqrt(1-sq(C3(x,y,z)));
  }
  else if(Second_Solution==true){//down elbow solution
    return -sqrt(1-sq(C3(x,y,z)));
  }
}

//Angle Th1(base)
double Th_1(int x,int y) {
  if (C3_Error==false){//if sqrt limitations are not violated..
   return Rad_To_Deg(atan2 (y,x)); //reverse x and y cause the library is reversed 
  }
}

//Angle Th2
double Th_2(int x, int y, int z, bool Second_Solution=false)
{
  if (C3_Error==false){//if sqrt limitations are not violated..
   return Rad_To_Deg(atan2(z-l1, sqrt(sq(x)+ sq(y))) - atan2(l3*S3(x,y,z,Second_Solution), l2+(l3*C3(x,y,z)))); 
  }
}

//Angle Th3
double Th_3(int x,int y,int z,bool Second_Solution=false)
{
  if (C3_Error == false){//if sqrt limitations are not violated..
      return Rad_To_Deg(atan2(S3(x,y,z, Second_Solution),C3(x,y,z)));
  }
}

//Function that checks the physical limitations of th1, th2, th3 angles. i needed to use a function, because i will use it twice every time we calculate the angles (up albow solution and down elbow solution)
void Physical_Limitations_Check(double Th2_Calc, double Th3_Calc)
{
     //in case you are using a different arm, you should change te physical angle range limitations
     //physical limitations should change according to calibration settings
      if(Th2_Calc<(Th2_max_negative) || Th2_Calc>(Th3_max_positive))//if there is a physical limitation, then Ph_Range_error=true
      {
        Th2_Ph_Range_Error=true;
        Serial.println("Th2_Calc range violation");
      }
      else{
        Th2_Ph_Range_Error=false;
      }

      if(Th3_Calc< (Th3_max_negative) || Th3_Calc>(Th2_max_positive))
      {
        Th3_Ph_Range_Error=true;
        Serial.println("Th3_Calc range violation");
      }
      else
      {
        Th3_Ph_Range_Error=false;
      }
}

//OUR MAIN Function where we will give the coordinates and will unite all the Th_ functions to save the desired degrees
void Inverse_Calc(int x,int y,int z)
{
    if(led_state==1){
      digitalWrite(led,0);
      led_state=0;
}

  //checking if C3(x,y,z)'s limitations are not violated (c3 has a limitation due to sqrt)
   C3(x,y,z);// if there will be an error, C3_Error variable will change to true.
   
   if(C3_Error == false){//if the c3 limitation is not violated (due to S3 sqrt)...
//Save each degree values to the corresponding variables. Solve for both up elbow solution and down elbow solution (2nd slution)
//first solution
      Th1_Calc_1=Th_1(x,y); 
      Th2_Calc_1=Th_2(x,y,z);
      Th3_Calc_1=Th_3(x,y,z);
      
   }
   else{
    Serial.println("C3_Error occured, check your input. C3_error is not supposed to stop the program, but continue executing it");
    Serial.println("=============================Done With C3_Error==============================");
    delay(100);
    
    if(led_state==0){
      digitalWrite(led,1);
      led_state=1;
    }
    return(1); //if there is an C3_Error, this function will exit here.
   }
  
   
  Physical_Limitations_Check(Th2_Calc_1, Th3_Calc_1);
 
   if ((Th2_Ph_Range_Error==false) && (Th3_Ph_Range_Error==false) && y>=0){//if there is no error, continue..     
          Serial.print("Th1_Calc_1= ");
          Serial.println(Th1_Calc_1);
          Serial.print("Th2_Calc_1= ");
          Serial.println(Th2_Calc_1);
          Serial.print("Th3_Calc_1= ");
          Serial.println(Th3_Calc_1);  
          Servo_Mov(Th1_Calc_1, Th2_Calc_1, Th3_Calc_1 );
   }
  else{
    Serial.println("Limitations Violated for first solution");

    Th2_Ph_Range_Error=false;
    Th3_Ph_Range_Error=false;
    
      Serial.println("we can Check second solution");
      
      //second solution
      Th1_Calc_2=Th_1(x,y); 
      Th2_Calc_2=Th_2(x,y,z,true);
      Th3_Calc_2=Th_3(x,y,z,true);
      
      Physical_Limitations_Check(Th2_Calc_2, Th3_Calc_2);
      
      if ((Th2_Ph_Range_Error==false) && (Th3_Ph_Range_Error==false) && y>=0){//if there is no error, continue..  
      Serial.print("Th1_Calc_2= ");
      Serial.println(Th1_Calc_2);
      Serial.print("Th_Calc_2= ");
      Serial.println(Th2_Calc_2);
      Serial.print("Th3_Calc_2= ");
      Serial.println(Th3_Calc_2); 
        
        Servo_Mov(Th1_Calc_2, Th2_Calc_2, Th3_Calc_2 );
    }
    else {
      Serial.println("Physical limitations cannot cope with 1st or 2nd solution");
      digitalWrite(led,1);
    }
}
      C3_Error= false;
      Th2_Ph_Range_Error=false;
      Th3_Ph_Range_Error=false;
      Serial.println("=============================Done==============================");
}


//Calibration and Function that helps the servos go smoothly from one position to another. We need this so that servos wont move violently from one position to another
void Servo_Mov(float Deg1, float Deg2, float Deg3){
  //finding the difference-distance/steps- each arm has to do
   Final_Deg1= TH1(Deg1);
   Diff1= Th1-Final_Deg1;

   Final_Deg2= TH2(Deg2);
   Diff2= Th2-Final_Deg2;

   Final_Deg3= TH3(Deg3);
   Diff3= Th3- Final_Deg3;

 //finding out which arm has the biggest difference.
 if(Diff1<0){
  UDiff1=-Diff1;
 }
 else{
  UDiff1=Diff1;
 }
 
 if(Diff2<0){
  UDiff2=-Diff2;
 }
 else{
  UDiff2=Diff2;
 }
 
 if(Diff3<0){
  UDiff3=-Diff3;
 }
 else{
  UDiff3=Diff3;
 }
 
 float comparison1= max(UDiff1, UDiff2);
 float Maximum=max(comparison1,UDiff3);

//  Serial.print("Maximum= ");
//  Serial.println(Maximum);
  
 Diff1= Diff1/Maximum;
 Diff2= Diff2/Maximum;
 Diff3= Diff3/Maximum;

  
  for(i=0; i<=Maximum; i++)
  {
    Ser_1.write(Th1 - Diff1);
    Ser_2.write(Th2 - Diff2);
    Ser_3.write(Th3 - Diff3);
    delay(delayTime);
    Th1=Th1 - Diff1;
    Th2=Th2 - Diff2;
    Th3=Th3 - Diff3;
  }
}




void setup() {
//declare which pin are my servos attached
Ser_1.attach(2);
Ser_2.attach(3);
Ser_3.attach(4);

//if ur using a led for error indicator
pinMode(led,OUTPUT);

delay(3000);
//Start serial port
Serial.begin(9600);

}

void loop() {
//Inverse_Calc(10,10,3);
}
