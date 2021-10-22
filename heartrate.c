#include <stdio.h>
#include <time.h>
#include "heartrate.h"


int hRcounter = 1;
float hRsum = 0;


int heartRateThresh(float hRdata){
   float avg;
   hRsum += hRdata; // new data point added to sum
   if(hRcounter < 20){ avg = hRsum / hRcounter;hRcounter++;}// if we have less than 20 data points in average: Compute average //////WORKS
  else{hRcounter++;}
   
   // otherwise compare each data point to average
    if((hRdata < (avg*0.9)) && hRcounter >= 20){ // 10% decrease
    printf("hR average is: %f\n",avg);
      printf("HR ALARMING VALUE\n");
      printf("Datapoint corresponding is: ");
      printf("%f\n",hRdata);

      printf("This ocurred on the %dth try\n",hRcounter);
      return 1;
   }

   return 0;

}

void generateHRData(){

   srand(0);

   for (int i = 0; i < 30; i++) { // generate 20 random numbers between 100 and 110
      int test = heartRateThresh((rand() % 10) + 100);
      
   }

   for(int i = 0; i < 30; i++){ // generate lower numbers beween 93 and 103
   int b =heartRateThresh((rand() % 10) + 93); 
   //int b = heartRateThresh(100);
   if(b == 1){break;}
   }

   return;
}