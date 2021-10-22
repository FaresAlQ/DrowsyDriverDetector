#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "heartrate.h"
#include "gyroscope.h"

// take input as single data point DONE

// has enough time elapsed to start checking DONE

// if it has then begin calculating DONE

// add to average DONE 

// make single comparisons everytime DONE

// check for edge cases each time






int main() {

   time_t begin,end;

    begin= time(NULL);

   while(1){ // wait to begin calculating
      end = time(NULL);
      //printf("waiting");
      if((difftime(end,begin)) > 3.0){
         break;
      }
      }
   
   // want to call every 5 seconds


   
   


    generateHRData();
    generateGyroData();
   




   return 0;
}