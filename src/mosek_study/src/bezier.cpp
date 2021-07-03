#include <stdio.h>
#include "bits/stdc++.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
using namespace std;
double evaluateBezierBase(int i,int n)
{
    double x=n;
    double fenmu=1;
    double j=n-i;
    while(x>0)
    {
        fenmu=fenmu*x;
        x--;
    }
    
    while(i>0)
    {
        fenmu=fenmu/double(i);
        i--;
    }
    
    
    while(j>0)
    {
        fenmu=fenmu/double(j);
        j--;
    }
    cout<<"base "<<endl;
    cout<<fenmu<<endl;
    return fenmu;
}
void calculateQ()
{
    Eigen::VectorXd w3(7);
    Eigen::VectorXd w4(7);
    Eigen::VectorXd w5(7);
    Eigen::VectorXd w6(7);
    w3<<-120,360,-360,120,0,0,0;
    w4<<360,-1440,2160,-1440,360,0,0;
    w5<<-360,1800,-3600,3600,-1800,360,0;
    w6<<120,-720,1800,-2400,1800,-720,120;
    Eigen::MatrixXd matr(7,7);
    matr=   w3*w3.transpose()   +w3*w4.transpose()/2+w3*w5.transpose()/3+w3*w6.transpose()/4+
            w4*w3.transpose()/2 +w4*w4.transpose()/3+w4*w5.transpose()/4+w4*w6.transpose()/5+
            w5*w3.transpose()/3 +w5*w4.transpose()/4+w5*w5.transpose()/5+w5*w6.transpose()/6+
            w6*w3.transpose()/4 +w6*w4.transpose()/4+w6*w5.transpose()/6+w6*w6.transpose()/7    ;
    cout<<setprecision(8);
    cout<<"matrix "<<endl;
    cout<<matr<<endl;
    // cout<<"matrix2 "<<endl;
    // cout<<w3*w4.transpose()<<endl;
}

int main()
{
    ofstream fs;
    fs.open("/home/dji/catkin_ws/src/ooqp_study/date/bezier_data.txt");
    double control_point[]={1,5,3,4,2,6,7};
    int control_num=sizeof(control_point)/sizeof(double);
    calculateQ();
    double base[control_num];
    for(int i=0;i<control_num;i++)
    {   
        base[i]=evaluateBezierBase(i,control_num-1);
        cout<< base[i]<<endl;
    }
    for(double t=0;t<1+0.01;t=t+0.01)
    {
        double bezier_i=0;
        for(int i=0;i<control_num;i++)
        {
            bezier_i+=control_point[i]*base[i]*pow(t,i)*pow(1-t,control_num-1-i);
        }
        double polynomial=0;
        polynomial+=control_point[0]+(-6*control_point[0]+6*control_point[1])*t+(15*control_point[0]-30*control_point[1]+15*control_point[2])*t*t+
                    (-20*control_point[0]+60*control_point[1]-60*control_point[2]+20*control_point[3])*t*t*t+(15*control_point[0]-60*control_point[1]+90*control_point[2]-
                    60*control_point[3]+15*control_point[4])*pow(t,4)+(-6*control_point[0]+30*control_point[1]-60*control_point[2]+60*control_point[3]-
                    30*control_point[4]+6*control_point[5])*pow(t,5)+(control_point[0]-6*control_point[1]+15*control_point[2]-20*control_point[3]+15*control_point[4]-
                    6*control_point[5]+control_point[6])*pow(t,6);
       // cout<<bezier_i<<endl;
        fs<<t<<" "<<bezier_i<<" "<<polynomial<<endl;
    }
    fs.close();

}