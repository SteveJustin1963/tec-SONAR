/// Track the object that emitted the sonar waves

#include <stdio.h>
#include <math.h>

int main()
{
    int v, d;
    float t;
    
    printf("Enter the velocity (in m/s) of the object: ");
    scanf("%d", &v);
    printf("Enter the distance (in m) of the object: ");
    scanf("%d", &d);
    
    t = (2 * d) / v;
    
    printf("The object is %.2f m away.\n", d);
    printf("It will take %.2f seconds for the object to reach the observer.\n", t);
    
    return 0;
}
///
