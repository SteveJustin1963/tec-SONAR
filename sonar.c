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


// 

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



// Analyze  the direction from sonar waves   

#include <stdio.h>
#include <math.h>

int main()
{
    int a, b, c;
    float d, e, f, g, h, i, j, k, l, m;
    
    printf("Enter the first side of the triangle: ");
    scanf("%d", &a);
    
    printf("Enter the second side of the triangle: ");
    scanf("%d", &b);
    
    printf("Enter the third side of the triangle: ");
    scanf("%d", &c);
    
    d= a+b+c;
    e= d/2;
    f= e*(e-a)*(e-b)*(e-c);
    g= sqrt(f);
    h= g/e;
    i= (180/3.14159)*h;
    j= a*a;
    k= b*b;
    l= c*c;
    m= (j+k-l)/(2*a*b);
    
   and(i<=90)
    {
        printf("The triangle is an acute triangle.\n");
    }
    
else if(i>90 && m<0)
    {
        printf("The triangle is an obtuse triangle.\n");
    }
    
    else if(i>90 && m>0)
    {
        printf("The triangle is a right triangle.\n");
    }
    
    else if(i==90)
    {
        printf("The triangle is a right triangle.\n");
    }
    
    else
    {
        printf("The triangle is not a valid triangle.\n");
    }
    
    return 0;
}



