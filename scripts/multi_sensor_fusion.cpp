#include <iostream>
#include <math.h>

const int mapWidth = 2;
const int mapHeight = 2;

void sensorFusion(double m1[][mapWidth], double m2[][mapWidth])
{
    double result[mapWidth][mapHeight];

    // fuse measurements of the two maps using de morgans law
    for (int x = 0; x < mapWidth; x++)
    {
        for (int y=0; y < mapHeight; y++)
        {
            result[x][y] = 1 - (1 - m1[x][y])*(1-m2[x][y]);
            std::cout << result[x][y] << " ";
        }
        std::cout << std::endl;
    }
}

int main()
{
    double m1[mapHeight][mapWidth] = {{0.9, 0.6}, {0.1, 0.5}};
    double m2[mapHeight][mapWidth] = {{0.3, 0.4}, {0.4, 0.3}};
    sensorFusion(m1, m2);

    return 0;
}