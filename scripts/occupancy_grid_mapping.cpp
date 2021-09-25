#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

// sensor characteristics
double Zmax = 5000;
double Zmin = 170;

// define cell state
double l0 = 0,locc = 0.4, lfree = -0.4;

// grid dimensions
double gridWidth = 100, gridHeight = 100;

// map dimensions
double mapWidth = 30000, mapHeight = 15000;

// robot size with respect to map
double robotXOffset = mapWidth/5, robotYOffset = mapHeight/3;

//define an l vector to store log odds values of each cell
std::vector<std::vector<double> > l(mapWidth/gridWidth,std::vector<double>(mapHeight/gridHeight));

double inverseSensorModel(double x, double y, double theta, double xi, double yi, double sensorData[])
{
    return 0.4;
}

void occupancyGridMapping(double Robotx, double Roboty, double Robottheta, double sensorData[])
{
    for (int x = 0; x < mapWidth/gridWidth; x++)
    {
        for (int y = 0; y < mapHeight/gridHeight; y++)
        {
            double xi = x * gridWidth + gridWidth/2 - robotXOffset;
            double yi = -(y * gridHeight + gridHeight/2) + robotYOffset;
            if (sqrt(pow(xi-Robotx, 2) + pow(yi-Roboty, 2)) <= Zmax)
            {
                l[x][y] = l[x][y] +inverseSensorModel(Robotx, Roboty, Robottheta, xi, yi, sensorData) - l0;
            }
        }
    }
}

int main()
{
    double timeStamp;
    double measurementData[8];
    double robotX, robotY, robotTheta;

    FILE* posesFile = fopen("poses.txt", "r");
    FILE* measurementFile = fopen("measurement.txt", "r");

    // scan files and retrieve measurement and poses
    while(fscanf(posesFile, "%lf %lf %lf %lf", &timeStamp, &robotX, &robotY, &robotTheta) != EOF)
    {
        fscanf(measurementFile, "%lf", &timeStamp);
        for (int i = 0; i < 8; i++)
        {
            fscanf(measurementFile,"%lf", &measurementData[i]);
        }
        occupancyGridMapping(robotX, robotY, (robotTheta/10) * (M_PI/180), measurementData);
    }

    // displaying the map
    for (int x = 0;x < mapWidth / gridWidth; x++)
    {
        for (int y = 0; y < mapHeight / gridHeight; y++)
        {
            std::cout << l[x][y] << " ";
        }
    }

    return 0;
}