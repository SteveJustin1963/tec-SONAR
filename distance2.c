 

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

