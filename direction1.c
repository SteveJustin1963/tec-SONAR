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



