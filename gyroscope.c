#include <stdio.h>
#include <time.h>
#include "gyroscope.h"




int gyroCt = 1;
float gyroSum = 0;

int gyroThresh(float angle){
   // WILL RETURN VALUE OF 1 FOR ALARMING VALUE

   if(angle >=90){return 1;} // for edge case
   float avg;
   gyroSum += angle;
   if(gyroCt < 20){ avg = gyroSum / gyroCt;gyroCt++;} // if we have less than 20 data points in average: Compute average //////WORKS
   else{
      // printf("Angle Average is:");
      // printf("%f \n",avg);
      gyroCt++;
   }
   // otherwise compare each data point to average
    if((angle > (avg*1.2)) && (gyroCt >= 20)){ // 10% decrease
      printf("ANGLE ALARMING VALUE\n");
      printf("angle corresponding is: ");
      printf("%f\n",angle);
      printf("This ocurred on the %dth try\n",gyroCt);
      printf("Angle average is: ");
      printf("%f\n",avg);
      return 1;
   }

   return 0;
}

void generateGyroData(){
   
   int  a;
   srand(0);
  int n = 2;
  int ct = 0;
  int milli_seconds=n*1000;
  time_t start, end1;
  start=time(0);
  
  
  //while(1){
    
 
   //  if(time(0)-start==n)
   //  {
   //     printf("called \n");
   //    a = gyroThresh((rand() % 10) + 20); // generate numbers 10 and 30
   //    if(a == 1){break;}
   //    start=start+n;
   //  }
   //  ct++;

   //  }

      // int b = gyroThresh(90);
      // if(b == 1){printf("caught it\n");}
     for (int i = 0; i < 20; i++) { // generate numbers between 20 and 30
      int a = gyroThresh((rand() % 10) + 20);
   }

   for(int i = 0; i < 30; i++){ // generate lower numbers beween 93 and 103
   //int b heartRateThresh((rand() % 10) + 93); 
   int a = gyroThresh(40);
   if(a == 1){break;}
   }


//   while(1){
    
//     if(time(0)-start==n)
//     {
//       a = gyroThresh((rand() % 60) + 10); // generate numbers 10 and 30
//       if(a == 1){break;}
//       start=start+n;
//     }
//   }




   return;
}
