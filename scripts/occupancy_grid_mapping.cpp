#include <iostream>
#include <algorithm>
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
std::vector< std::vector<double> > l(mapWidth/gridWidth,std::vector<double>(mapHeight/gridHeight));

double inverseSensorModel(double x, double y, double theta, double xi, double yi, double sensorData[])
{
    // define sensor characteristics
    double Zk, thetaK, sensorTheta;
    double minDelta = -1;
    double alpha = 200, beta = 20;

    // compute r and phi
    double r = sqrt(pow(xi-x,2)+pow(yi-y,2));
    double phi = atan2(yi-y, xi-x) - theta;

    // scaling measurement to [-90 -37.5 -22.5 -7.5 7.5 22.5 37.5 90]
    for (int i = 0; i < 8; i++)
    {
        if (i == 0)
        {
            sensorTheta = -90 * (M_PI / 180);
        }
        else if (i == 1)
        {
            sensorTheta = -37.5 * (M_PI / 180);
        }
        else if (i == 6)
        {
            sensorTheta = 37.5 * (M_PI / 180);
        }
        else if (i == 7)
        {
            sensorTheta = 90 * (M_PI / 180);
        }
        else
        {
            sensorTheta = (-37.5 + (i-1)*15)*(M_PI / 180);
        }

        if (fabs(phi - sensorTheta) < minDelta || minDelta == -1)
        {
            Zk = sensorData[i];
            thetaK = sensorTheta;
            minDelta = fabs(phi - sensorTheta);
        }
    }

    if (r > std::min(Zmax, Zk + alpha/2) || fabs(phi - thetaK) > beta/2 || Zk > Zmax || Zk < Zmin)
    {
        // return unknown
        return l0;
    }

    if (Zk < Zmax && fabs(r - Zk) < alpha/2)
    {
        // return occupied
        return locc;
    }

    if (r < Zk)
    {
        // return free
        return lfree;
    }

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