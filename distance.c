// calculate the distance to the object.
// We can use the following formula:
// Distance = (Speed of Sound * Time) / 2
// Distance is the distance to the object
// Speed of Sound is the speed of sound in the medium
// Time is the time it takes for the sound waves to bounce back
// Assuming that the speed of sound in the medium is 340 m/s, 
// we can write a program to calculate the distance to the object as follows:

#include <stdio.h>

int main()

{

  float distance;
  float speedOfSound;
  float time;

  speedOfSound = 340.0;

  printf("Enter the time it took for the sound waves to bounce back: ");
  scanf("%f", &time);

  distance = (speedOfSound * time) / 2.0;

  printf("The distance to the object is %.2f meters.", distance);

  return 0;

}


// or another prog

#include <stdio.h> 
#include <math.h>

int main() 
{ 

float dist; 
float time; 

printf("Enter the time taken for the sound waves to bounce back: "); 
scanf("%f", &time); 

dist = (time * 340) / 2; 

printf("The distance to the object is %f m.", dist); 

return 0; 
}

