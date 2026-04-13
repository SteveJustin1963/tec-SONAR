// Analyze the direction from sonar waves using triangle geometry.
// Given three range measurements (sides a, b, c) from sonar sensors,
// classify the resulting triangle to determine the bearing geometry.
//
// Classification uses the law of cosines applied to the largest side:
//   a^2 + b^2  > c^2  => acute  (all angles < 90 deg)
//   a^2 + b^2 == c^2  => right  (one angle exactly 90 deg)
//   a^2 + b^2  < c^2  => obtuse (one angle > 90 deg)

#include <stdio.h>

int main()
{
    int a, b, c, tmp;
    long long a2, b2, c2;

    printf("Enter the first side of the triangle:  ");
    scanf("%d", &a);
    printf("Enter the second side of the triangle: ");
    scanf("%d", &b);
    printf("Enter the third side of the triangle:  ");
    scanf("%d", &c);

    /* Validate triangle inequality */
    if (a <= 0 || b <= 0 || c <= 0 ||
        (a + b) <= c || (a + c) <= b || (b + c) <= a)
    {
        printf("Not a valid triangle.\n");
        return 1;
    }

    /* Rotate so c is the longest side (largest angle opposite c) */
    if (b > c) { tmp = b; b = c; c = tmp; }
    if (a > c) { tmp = a; a = c; c = tmp; }

    a2 = (long long)a * a;
    b2 = (long long)b * b;
    c2 = (long long)c * c;

    if (a2 + b2 == c2)
        printf("The triangle is a right triangle.\n");
    else if (a2 + b2 > c2)
        printf("The triangle is an acute triangle.\n");
    else
        printf("The triangle is an obtuse triangle.\n");

    return 0;
}



