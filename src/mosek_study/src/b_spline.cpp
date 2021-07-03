#include <stdio.h>
#include "bits/stdc++.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
using namespace std;
double evaluateDeBoor(int i,int p, double u,int *node){}
int main()
{
    double control_point[]={1,2,3,4,5,6,7};
    int control_num=sizeof(control_point)/sizeof(double);
    double t=2;
    double k=3;
    int node_m=k+control_num+1;
    double node[node_m]={0,0,0,0,  0.25,0.5,0.75,  2,2,2,2};
    for(double u=0;u<t;u+=0.05)
    {
        double b_result=0;
        for(int i=0;i<control_num;i++)
        {
            b_result+=control_point[i];
        }
    }

}