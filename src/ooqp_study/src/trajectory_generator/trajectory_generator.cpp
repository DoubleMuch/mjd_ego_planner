#include "trajectory_generator.h"
void trajectory_generator::waypoints_make_traj(vector<waypoint_info> waypoints,vector <traj_info>  &traj,int &flag)
{
    double R=0.6;
    double dis=0.4;

    int usage_ok = 1, quiet = 0;
    int traj_num=waypoints.size()-1;
    if(traj_num < 1)
    {
        cout<<"ERROR !!! Waypoints at least 2"<<endl;
        return ;
    }
    vector<double> time_slots;
    for(int i=0;i<traj_num;i++)
    {
        time_slots.push_back(waypoints[i+1].time-waypoints[i].time);
    }

    int jerk_n=6;  //minimum jerk 系数
    int domin=3;   //x,y,z维度3
    int nx=jerk_n*domin*traj_num;   //总共的系数个数
    double c[nx]={0};           //线性项
    double xupp[nx] = {0};  
    char ixupp[nx] = {0};

    double xlow[nx] = {0};  
    char ixlow[nx] = {0}; 

    double end_time=time_slots[traj_num-1];
    const int nnzQ = 6*3*traj_num;       // 数目与矩阵的下三角矩阵的个数相对应
    int irowQ[6*domin*traj_num];
    int jcolQ[6*domin*traj_num];
    double dQ[6*domin*traj_num];
    int countQ=0;
    int line_offset_Q=0;
    for(int i=0;i<traj_num;i++)
    {
        // for(int j=0;j<domin;j++)
        // {
            //x
            irowQ[countQ]=i*18+3;
            jcolQ[countQ]=i*18+3;
            dQ[countQ]=36*time_slots[i];
            countQ++;

            irowQ[countQ]=i*18+4;
            jcolQ[countQ]=i*18+3;
            dQ[countQ]=72*pow(time_slots[i],2);
            countQ++;
            irowQ[countQ]=i*18+4;
            jcolQ[countQ]=i*18+4;
            dQ[countQ]=192*pow(time_slots[i],3);
            countQ++;

            irowQ[countQ]=i*18+5;
            jcolQ[countQ]=i*18+3;
            dQ[countQ]=60*pow(time_slots[i],3);
            countQ++;
            irowQ[countQ]=i*18+5;
            jcolQ[countQ]=i*18+4;
            dQ[countQ]=360*pow(time_slots[i],4);
            countQ++;
            irowQ[countQ]=i*18+5;
            jcolQ[countQ]=i*18+5;
            dQ[countQ]=720*pow(time_slots[i],5);
            countQ++;

            //y
            irowQ[countQ]=i*18+9;
            jcolQ[countQ]=i*18+9;
            dQ[countQ]=36*time_slots[i];
            countQ++;
            
            irowQ[countQ]=i*18+10;
            jcolQ[countQ]=i*18+9;
            dQ[countQ]=72*pow(time_slots[i],2);
            countQ++;
            irowQ[countQ]=i*18+10;
            jcolQ[countQ]=i*18+10;
            dQ[countQ]=192*pow(time_slots[i],3);
            countQ++;

            irowQ[countQ]=i*18+11;
            jcolQ[countQ]=i*18+9;
            dQ[countQ]=60*pow(time_slots[i],3);
            countQ++;
            irowQ[countQ]=i*18+11;
            jcolQ[countQ]=i*18+10;
            dQ[countQ]=360*pow(time_slots[i],4);
            countQ++;
            irowQ[countQ]=i*18+11;
            jcolQ[countQ]=i*18+11;
            dQ[countQ]=720*pow(time_slots[i],5);
            countQ++;

            //z
            double gain=10;
            irowQ[countQ]=i*18+15;
            jcolQ[countQ]=i*18+15;
            dQ[countQ]=gain*36*time_slots[i];
            countQ++;
            
            irowQ[countQ]=i*18+16;
            jcolQ[countQ]=i*18+15;
            dQ[countQ]=gain*72*pow(time_slots[i],2);
            countQ++;
            irowQ[countQ]=i*18+16;
            jcolQ[countQ]=i*18+16;
            dQ[countQ]=gain*192*pow(time_slots[i],3);
            countQ++;

            irowQ[countQ]=i*18+17;
            jcolQ[countQ]=i*18+15;
            dQ[countQ]=gain*60*pow(time_slots[i],3);
            countQ++;
            irowQ[countQ]=i*18+17;
            jcolQ[countQ]=i*18+16;
            dQ[countQ]=gain*360*pow(time_slots[i],4);
            countQ++;
            irowQ[countQ]=i*18+17;
            jcolQ[countQ]=i*18+17;
            dQ[countQ]=gain*720*pow(time_slots[i],5);
            countQ++;

        // }
    }
    for(int i=0;i<6*domin*traj_num;i++)
    {
        dQ[i]=dQ[i]*1;
    }
    // int irowQ[] = {3,   
    //                 4,  4,  
    //                 5,  5,  5,
    //                 9,   
    //                 10, 10, 
    //                 11, 11, 11,
    //                 15, 
    //                 16, 16,
    //                 17, 17, 17};  // 对称矩阵Q，仅在irowQ,jcolQ和dQ中指定
    // int jcolQ[] = { 3,  
    //                 3,  4,  
    //                 3,  4,  5,
    //                 9,  
    //                 9,  10, 
    //                 9,  10, 11,
    //                 15, 
    //                 15, 16, 
    //                 15, 16, 17 };  // 矩阵的下三角元素
    // double dQ[] = {2*36*end_time,
    //                2*72*pow(end_time,2),  2*192*pow(end_time,3),
    //                2*60*pow(end_time,3), 2*360*pow(end_time,4),    2*720*pow(end_time,5),
    //                2*36*end_time,
    //                2*72*pow(end_time,2),  2*192*pow(end_time,3),
    //                2*60*pow(end_time,3), 2*360*pow(end_time,4),    2*720*pow(end_time,5),
    //                2*36*end_time,
    //                2*72*pow(end_time,2),  2*192*pow(end_time,3),
    //                2*60*pow(end_time,3), 2*360*pow(end_time,4),    2*720*pow(end_time,5)};





    
    int my=18;      //首尾状态等式约束 
    my+=3*3*(traj_num-1); //中间航点连续性约束
    my+=(traj_num-1)*3;    //中间航点位置约束
    my+=2;

    cout<<"my   "<< my<<endl;
    //A矩阵，先描述首尾状态等式，然后描述中间状态位置，速度，加速度连续等式
    double b[my]={0};     
    int nnzA =18*3;             // AX=b矩阵A中non-zeros总数
    nnzA+=18*3*(traj_num-1);    //位置，速度，加速度连续性约束
    nnzA+=3*(traj_num-1);       //位置点约束
    nnzA+=2;                    //姿态控制点速度，加速度约束
    cout<<"nnzA "<<nnzA<<endl;
    
    b[0]=waypoints[0].w_pos[0];b[1]=0;b[2]=0;  //第一个点的px,vx,ax约束
    b[3]=waypoints[traj_num].w_pos[0];b[4]=0;b[5]=0;  //最后一个点的px,vx,ax约束

    b[6]=waypoints[0].w_pos[1];b[7]=0;b[8]=0;  //第一个点的py,vy,ay约束
    b[9]=waypoints[traj_num].w_pos[1];b[10]=0;b[11]=0;  //最后一个点的py,vy,zy约束

    b[12]=waypoints[0].w_pos[2];b[13]=0;b[14]=0;  //第一个点的pz,vz,az约束
    b[15]=waypoints[traj_num].w_pos[2];b[16]=0;b[17]=0;  //最后一个点的pz,vz,az约束

    //剩余的为连续性相关等式约束，Xi(t)=Xi+1(t)类型，可以转化为Xi(t)-Xi+1(t)=0；等式的值为0
    double last_traj_col_offset=(traj_num-1)*18;
    //double end_time=time_slots[traj_num-1];
    int irowA[nnzA]={0,
                        1,
                            2,
                     3, 3,  3,  3,  3,  3,
                        4,  4,  4,  4,  4,
                            5,  5,  5,  5,
                    6,
                        7,
                            8,
                     9, 9,  9,  9,  9,  9,
                        10,  10,  10,  10,  10,
                            11,  11,  11,  11,
                    12,
                        13,
                            14,
                    15, 15,  15,  15,  15,  15,
                        16,  16,  16,  16,  16,
                            17,  17,  17,  17};            // 等式约束中的A
                            
    int jcolA[nnzA]={0,
                        1,
                            2,
                    last_traj_col_offset+0,  last_traj_col_offset+1,  last_traj_col_offset+2,  last_traj_col_offset+3,  last_traj_col_offset+4,  last_traj_col_offset+5,
                        last_traj_col_offset+1,  last_traj_col_offset+2,  last_traj_col_offset+3,  last_traj_col_offset+4,  last_traj_col_offset+5,
                            last_traj_col_offset+2,  last_traj_col_offset+3,  last_traj_col_offset+4,  last_traj_col_offset+5,
                    6,
                        7,
                            8,
                    last_traj_col_offset+6,  last_traj_col_offset+7,  last_traj_col_offset+8,  last_traj_col_offset+9,  last_traj_col_offset+10,  last_traj_col_offset+11,
                        last_traj_col_offset+7,  last_traj_col_offset+8,  last_traj_col_offset+9,  last_traj_col_offset+10,  last_traj_col_offset+11,
                            last_traj_col_offset+8,  last_traj_col_offset+9,  last_traj_col_offset+10,  last_traj_col_offset+11,
                    12,
                        13,
                            14,
                    last_traj_col_offset+12, last_traj_col_offset+13, last_traj_col_offset+14, last_traj_col_offset+15, last_traj_col_offset+16, last_traj_col_offset+17,
                        last_traj_col_offset+13, last_traj_col_offset+14, last_traj_col_offset+15, last_traj_col_offset+16, last_traj_col_offset+17,
                            last_traj_col_offset+14, last_traj_col_offset+15, last_traj_col_offset+16, last_traj_col_offset+17}; 
                            
    double dA[nnzA]={1,
                        1,
                            1,
                    1,end_time,pow(end_time,2),pow(end_time,3),pow(end_time,4),pow(end_time,5),
                        1,  2*end_time, 3*pow(end_time,2),4*pow(end_time,3),5*pow(end_time,4),
                            2,  6*end_time, 12*pow(end_time,2), 20*pow(end_time,3),
                    1,
                        1,
                            1,
                    1,end_time,pow(end_time,2),pow(end_time,3),pow(end_time,4),pow(end_time,5),
                        1,  2*end_time, 3*pow(end_time,2),4*pow(end_time,3),5*pow(end_time,4),
                            2,  6*end_time, 12*pow(end_time,2), 20*pow(end_time,3),
                    1,
                        1,
                            1,
                    1,end_time,pow(end_time,2),pow(end_time,3),pow(end_time,4),pow(end_time,5),
                        1,  2*end_time, 3*pow(end_time,2),4*pow(end_time,3),5*pow(end_time,4),
                            2,  6*end_time, 12*pow(end_time,2), 20*pow(end_time,3)};
    int tem_irowA[54]={
        0,  0,  0,  0,  0,  0,  0,
            1,  1,  1,  1,  1,  1,
                2,  2,  2,  2,  2
    };
    int tem_jcolA[54]={
        0,  1,  2,  3,  4,  5,  18,
            1,  2,  3,  4,  5,  19,
                2,  3,  4,  5,  20
    };
    double tem_dA[54]={
        1,  1,  1,  1,  1,  1,  -1,
            1,  2,  3,  4,  5,  -1,
                2,  6,  12, 20, -2
    };
    int counter=54;
    int line_offset=18;
    //位置，速度，加速度，连续性约束
    for(int i=0;i<(traj_num-1);i++)
    {
        double end_t=time_slots[i];
        for(int j=0;j<domin;j++)
        {
            dA[counter]=1;dA[counter+1]=end_t;dA[counter+2]=pow(end_t,2);dA[counter+3]=pow(end_t,3);dA[counter+4]=pow(end_t,4);dA[counter+5]=pow(end_t,5);dA[counter+6]=-1;
            dA[counter+7]=1;dA[counter+8]=2*end_t;dA[counter+9]=3*pow(end_t,2);dA[counter+10]=4*pow(end_t,3);dA[counter+11]=5*pow(end_t,4);dA[counter+12]=-1;
            dA[counter+13]=2;dA[counter+14]=6*end_t;dA[counter+15]=12*pow(end_t,2);dA[counter+16]=20*pow(end_t,3);dA[counter+17]=-2;
            for(int k=0;k<18;k++)
            {
                irowA[counter]=line_offset+i*9+j*3+tem_irowA[k];
                jcolA[counter]=i*18+j*6+tem_jcolA[k];
                counter++;
            }

            
        }
        
    }

    //中间航点位置约束
    line_offset+=(traj_num-1)*9;
    for(int i=0;i<(traj_num-1);i++)
    {
        irowA[counter]=line_offset;
        jcolA[counter]=(i+1)*18;
        dA[counter]=1;
        b[line_offset]=waypoints[i+1].w_pos[0];
        counter++;
        line_offset++;


        irowA[counter]=line_offset;
        jcolA[counter]=(i+1)*18+6;
        dA[counter]=1;
        b[line_offset]=waypoints[i+1].w_pos[1];
        counter++;
        line_offset++;
        
        
        irowA[counter]=line_offset;
        jcolA[counter]=(i+1)*18+12;
        dA[counter]=1;
        b[line_offset]=waypoints[i+1].w_pos[2];
        counter++;
        line_offset++;
    }


    //姿态控制点速度，加速度约束,vx=0;ay=0;
    b[line_offset]=0;
    irowA[counter]=line_offset;
    jcolA[counter]=37;
    dA[counter]=1;
    counter++;
    line_offset++;

    b[line_offset]=0;
    irowA[counter]=line_offset;
    jcolA[counter]=44;
    dA[counter]=2;
    counter++;
    line_offset++;


    cout<<"line_offset "<<line_offset<<endl;
    cout<<"counter "<<counter<<endl;





    

    double max_v=3;
    double max_acc=6;
    
    double temp=sqrt(R*R-dis*dis)/dis;
    const int mz = 7;         // 不等式约束的个数   

    double clow[] = {temp*g/2,   -max_v,-max_acc,-max_v,-max_acc,-max_v,-4.5};
    char iclow[] = {1,        1,1,1,1,1,1};
    // double clow[] = {0};
    // char iclow[] = {0};

    double cupp[] = {0,        max_v,max_acc,max_v,max_acc,max_v,5};
    char icupp[] ={0,            1,1,1,1,1,1};
    // double cupp[] = {-g};
    // char icupp[] ={1};

    const int nnzC = 8;       // 多项式不等式约束的个数
    int off_set=-1;
    int irowC[] = {0,0,         2+off_set, 3+off_set , 4+off_set,5+off_set ,6+off_set ,7+off_set};
    int jcolC[] =  {38,50,      37, 38, 43,44,49,50};
    double dC[] = {1,-temp,      1,2,1,2,1,2};


    QpGenSparseMa27 *qp = new QpGenSparseMa27(nx, my, mz, nnzQ, nnzA, nnzC);
    cout<<1<<endl;
    QpGenData *prob = (QpGenData *)qp->copyDataFromSparseTriple(
        c, irowQ, nnzQ, jcolQ, dQ, xlow, ixlow, xupp, ixupp, irowA, nnzA, jcolA,
        dA, b, irowC, nnzC, jcolC, dC, clow, iclow, cupp, icupp);
    
    QpGenVars *vars = (QpGenVars *)qp->makeVariables(prob);
    QpGenResiduals *resid = (QpGenResiduals *)qp->makeResiduals(prob);

    GondzioSolver *s = new GondzioSolver(qp, prob);

    cout<<1<<endl;
    if (!quiet)
    {
        
        s->monitorSelf();
        
    }
    cout<<33<<endl;
    struct timeval tv;
    gettimeofday(&tv,NULL);//获取1970-1-1到现在的时间结果保存到tv中
    double t1=tv.tv_sec*1000000+tv.tv_usec;
    int ierr = s->solve(prob, vars, resid);
    gettimeofday(&tv,NULL);//获取1970-1-1到现在的时间结果保存到tv中
    cout<<"usec "<<tv.tv_sec*1000000+tv.tv_usec-t1<<endl;
    cout<<4<<endl;
    double result[nx];
    if (ierr == 0) {
        cout.precision(4);
        cout << "Solution: \n";
        //cout<<vars->value<<endl;
        vars->x->copyIntoArray(result);
        
        
        vars->x->writefToStream(cout, "x[%{index}] = %{value}");
    } else {
        cout << "Could not solve the problem.\n";
        flag=0;
        return ;
    }

    for(int j=0;j<traj_num;j++)
    {
        for(int i=0;i<jerk_n;i++)
        {
            traj[j].x_coeff[i]=result[j*18+i]; 
        }
        for(int i=jerk_n;i<2*jerk_n;i++)
        {
            traj[j].y_coeff[i-jerk_n]=result[j*18+i]; 
        }
        for(int i=2*jerk_n;i<3*jerk_n;i++)
        {
            traj[j].z_coeff[i-2*jerk_n]=result[j*18+i]; 
        }
    }
}
