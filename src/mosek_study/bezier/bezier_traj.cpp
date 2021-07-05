#include "bezier_traj.h"
static void MSKAPI printstr(void *handle,
                            const char str[])
{
  printf("%s", str);
} /* printstr */
void bezier_traj::waypoints_bezier_traj(vector<waypoint_info> waypoints,vector <traj_info>  &traj,int &flag){
    int traj_num=waypoints.size()-1;
    int jerk_n=7;  //控制点个数7，多项式最高次数6
    int domin=3;   //x,y,z维度3
    int nx=jerk_n*domin*traj_num;   //总共的系数个数
    double R=0.4;
    double L=0.3;
    double temp=sqrt(R*R-L*L)/L;
    cout<<"roll degree "<<acos(L/R)*180/M_PI<<endl;
    MSKrescodee  r;
            //  首尾约束    中间航点位置约束     中间航点连续性约束     速度控制点约束                  加速度控制点约束
    int NUMCON =18      +(traj_num-1)*3     +(traj_num-1)*3*3   +traj_num*(jerk_n-1)*3      +traj_num*(jerk_n-2)*3
            //  窄点加速度约束      窄点速度约束
                +1;//              +1  ;       /* Number of constraints.             */
    int NUMVAR =7*3*traj_num;   /* Number of variables.               */
    int NUMANZ =0;   /* Number of non-zeros in A.          */
    int NUMQNZ =0;  /* Number of non-zeros in Q.          */
    if(traj_num < 1)
    {
        cout<<"ERROR !!! Waypoints at least 2"<<endl;
        return ;
    }
    vector<double> time_slots;
    for(int i=0;i<traj_num;i++)
    {
        time_slots.push_back(waypoints[i+1].time-waypoints[i].time);
        cout<<"time slots "<<time_slots[i]<<endl;
    }

    // formulation 1/2xTQx+cTx
    

    double        c[nx]={0};        //代价函数中的线性项
    //带求解变量x的各个单独约束,这里也是控制点的约束，控制点位置约束，
    MSKboundkeye  bkx[nx] = {MSK_BK_FR};
    for(int i=0;i<nx;i++)
        bkx[i]=MSK_BK_FR;
    double        blx[nx] = {-MSK_INFINITY};
    double        bux[nx] = { +MSK_INFINITY};



    //线性不等式约束中的上下界
    double matrixA[NUMCON][NUMVAR]={0};
    int NumEquCon=18      +(traj_num-1)*3     +(traj_num-1)*3*3;//等式约束个数
    //枚举类型的初始化能采用类似数组的统一初始化，下面那个for循环必不可少，要单独赋值
    MSKboundkeye  bkc[NUMCON] = {MSK_BK_FX};
    // for(int i=0;i<NumEquCon;i++)
    //     bkc[i]=MSK_BK_FX;
    double        blc[NUMCON] = {0};
    double        buc[NUMCON] = {0};
    // blc[0]=waypoints[0].w_pos[0]; buc[0]=waypoints[0].w_pos[0];        //waypoint_0_x
    // blc[3]=waypoints[0].w_pos[1]; buc[3]=waypoints[0].w_pos[1];       //waypoint_0_y
    // blc[6]=waypoints[0].w_pos[2]; buc[6]=waypoints[0].w_pos[2];      //waypoint_0_z
    // blc[9]=waypoints[traj_num].w_pos[0];    buc[9]=waypoints[traj_num].w_pos[0];        //waypoint_end_x 
    // blc[12]=waypoints[traj_num].w_pos[1];   buc[12]=waypoints[traj_num].w_pos[1];       //waypoint_end_y
    // blc[15]=waypoints[traj_num].w_pos[2];   buc[15]=waypoints[traj_num].w_pos[2];      //waypoint_end_z
    int row_index=0;
    //首尾位置约束的系数
    for(int i=0;i<3;i++)
    {
        // 起点位置 c0T0
        matrixA[row_index][i*7]=time_slots[0]*time_slots[0];
        blc[row_index]=waypoints[0].w_pos[i]; buc[row_index]=waypoints[0].w_pos[i]; bkc[row_index]=MSK_BK_FX;        //waypoint_0
        row_index++;
        //起点速度6(c1-c0)
        matrixA[row_index][i*7+1]=6;
        matrixA[row_index][i*7]=-6; bkc[row_index]=MSK_BK_FX;row_index++;
        //起点加速度30/T0(c2-2c1+c0)
        matrixA[row_index][i*7+2]=30/time_slots[0];
        matrixA[row_index][i*7+1]=-2*30/time_slots[0];
        matrixA[row_index][i*7+0]=30/time_slots[0]; bkc[row_index]=MSK_BK_FX;row_index++;

        NUMANZ+=6;
    }
    for(int i=0;i<3;i++)
    {
        // 终点位置 c6*Ttarj_num
        matrixA[row_index][(traj_num-1)*21+i*7+6]=time_slots[traj_num-1]*time_slots[traj_num-1];
        blc[row_index]=waypoints[traj_num].w_pos[i]; buc[row_index]=waypoints[traj_num].w_pos[i]; bkc[row_index]=MSK_BK_FX;        //waypoint_end
        row_index++;
        //终点速度6(c6-c5)
        matrixA[row_index][(traj_num-1)*21+i*7+6]=6;
        matrixA[row_index][(traj_num-1)*21+i*7+5]=-6; bkc[row_index]=MSK_BK_FX;row_index++;
        //终点加速度30/T0(c6-2c5+c4)
        matrixA[row_index][(traj_num-1)*21+i*7+6]=30/time_slots[traj_num-1];
        matrixA[row_index][(traj_num-1)*21+i*7+5]=-2*30/time_slots[traj_num-1];
        matrixA[row_index][(traj_num-1)*21+i*7+4]=30/time_slots[traj_num-1]; bkc[row_index]=MSK_BK_FX;row_index++;
        NUMANZ+=6;
    }
    

    //中间航点的坐标、位置连续、速度连续、加速度连续约束
    for(int i=0;i<(traj_num-1);i++)
    {
        for(int j=0;j<3;j++)
        {
            //位置坐标
            matrixA[row_index][21*i+j*7+6]=time_slots[i]*time_slots[i];   //c0t0=p0
            blc[row_index]=waypoints[i+1].w_pos[j];  
            buc[row_index]=waypoints[i+1].w_pos[j];
            bkc[row_index]=MSK_BK_FX;
            row_index++;

            //位置连续
            matrixA[row_index][21*i+j*7+6]=time_slots[i]*time_slots[i];
            matrixA[row_index][21*(i+1)+j*7]=-time_slots[i+1]*time_slots[i+1];
            bkc[row_index]=MSK_BK_FX;
            row_index++;

            //速度连续
            matrixA[row_index][21*i+j*7+6]=1*time_slots[i];
            matrixA[row_index][21*i+j*7+5]=-1*time_slots[i];
            matrixA[row_index][21*(i+1)+j*7+1]=-1*time_slots[i+1];
            matrixA[row_index][21*(i+1)+j*7+0]=1*time_slots[i+1];
            bkc[row_index]=MSK_BK_FX;
            row_index++;
            
            //加速度连续
            matrixA[row_index][21*i+j*7+6]=30;
            matrixA[row_index][21*i+j*7+5]=-2*30;
            matrixA[row_index][21*i+j*7+4]=30;
            matrixA[row_index][21*(i+1)+j*7+2]=-30;
            matrixA[row_index][21*(i+1)+j*7+1]=2*30;
            matrixA[row_index][21*(i+1)+j*7+0]=-30;
            bkc[row_index]=MSK_BK_FX;
            row_index++;

            NUMANZ+=13;
        }
    }
    
    double v_max=2;
    double a_max=100000;
    //速度控制点动力学约束
    for(int i=0;i<traj_num;i++)
    {
        for(int j=0;j<3;j++)
        {
            matrixA[row_index][i*21+j*7+1]=6*time_slots[i];
            matrixA[row_index][i*21+j*7+0]=-6*time_slots[i];
            blc[row_index]=-v_max; buc[row_index]=+v_max; bkc[row_index]=MSK_BK_RA;row_index++; //速度贝塞尔曲线的控制点约束范围，速度贝塞尔曲线有6个控制点，

            matrixA[row_index][i*21+j*7+2]=6*time_slots[i];
            matrixA[row_index][i*21+j*7+1]=-6*time_slots[i];
            blc[row_index]=-v_max; buc[row_index]=+v_max; bkc[row_index]=MSK_BK_RA;row_index++; 

            matrixA[row_index][i*21+j*7+3]=6*time_slots[i];
            matrixA[row_index][i*21+j*7+2]=-6*time_slots[i];
            blc[row_index]=-v_max; buc[row_index]=+v_max; bkc[row_index]=MSK_BK_RA;row_index++; 

            matrixA[row_index][i*21+j*7+4]=6*time_slots[i];
            matrixA[row_index][i*21+j*7+3]=-6*time_slots[i];
            blc[row_index]=-v_max; buc[row_index]=+v_max; bkc[row_index]=MSK_BK_RA;row_index++; 

            matrixA[row_index][i*21+j*7+5]=6*time_slots[i];
            matrixA[row_index][i*21+j*7+4]=-6*time_slots[i];
            blc[row_index]=-v_max; buc[row_index]=+v_max; bkc[row_index]=MSK_BK_RA;row_index++; 
    
            matrixA[row_index][i*21+j*7+6]=6*time_slots[i];
            matrixA[row_index][i*21+j*7+5]=-6*time_slots[i];
            blc[row_index]=-v_max; buc[row_index]=+v_max; bkc[row_index]=MSK_BK_RA;row_index++; 

            NUMANZ+=12;
        }
    }
    for(int i=0;i<traj_num;i++)
    {
        for(int j=0;j<3;j++)
        {
            matrixA[row_index][i*21+j*7+2]=30;
            matrixA[row_index][i*21+j*7+1]=-60;
            matrixA[row_index][i*21+j*7+0]=30;
            blc[row_index]=-a_max; buc[row_index]=+a_max; bkc[row_index]=MSK_BK_RA;
            if(j==2)
                blc[row_index]=-g;
            row_index++; //加速度贝塞尔曲线的控制点约束范围，加速度贝塞尔曲线有5个控制点，
            
            matrixA[row_index][i*21+j*7+3]=30;
            matrixA[row_index][i*21+j*7+2]=-60;
            matrixA[row_index][i*21+j*7+1]=30;
            blc[row_index]=-a_max; buc[row_index]=+a_max; bkc[row_index]=MSK_BK_RA;
            if(j==2)
                blc[row_index]=-g;
            row_index++;

            matrixA[row_index][i*21+j*7+4]=30;
            matrixA[row_index][i*21+j*7+3]=-60;
            matrixA[row_index][i*21+j*7+2]=30;
            blc[row_index]=-a_max; buc[row_index]=+a_max; bkc[row_index]=MSK_BK_RA;
            if(j==2)
                blc[row_index]=-g;
            row_index++;
        
            matrixA[row_index][i*21+j*7+5]=30;
            matrixA[row_index][i*21+j*7+4]=-60;
            matrixA[row_index][i*21+j*7+3]=30;
            blc[row_index]=-a_max; buc[row_index]=+a_max; bkc[row_index]=MSK_BK_RA;
            if(j==2)
                blc[row_index]=-g;
            row_index++;

            matrixA[row_index][i*21+j*7+6]=30;
            matrixA[row_index][i*21+j*7+5]=-60;
            matrixA[row_index][i*21+j*7+4]=30;
            blc[row_index]=-a_max; buc[row_index]=+a_max; bkc[row_index]=MSK_BK_RA;
            if(j==2)
                blc[row_index]=-g;
            row_index++;

            NUMANZ+=15;
        }
    }

    //狭窄航点的滚转角姿态约束转换到加速度上
    int narrow_index=1;         //狭窄航点的下标
    matrixA[row_index][narrow_index*21+2]=1;
    matrixA[row_index][narrow_index*21+1]=-2;
    matrixA[row_index][narrow_index*21+0]=1;
    matrixA[row_index][narrow_index*21+7*2+2]=-temp;
    matrixA[row_index][narrow_index*21+7*2+1]=2*temp;
    matrixA[row_index][narrow_index*21+7*2+0]=-temp;
    blc[row_index]=temp*g/30; buc[row_index]=+MSK_INFINITY; bkc[row_index]=MSK_BK_LO;row_index++;

    // //狭窄航点的水平速度约束
    // matrixA[row_index][narrow_index*21+1]=6;
    // matrixA[row_index][narrow_index*21+0]=-6;
    // blc[row_index]=0; buc[row_index]=0; bkc[row_index]=MSK_BK_FX;row_index++;
    
    
    //线性不等式约束的系数稀疏矩阵
    //不为0的系数在矩阵中的位置
    // MSKint32t   aptrb[NUMVAR]={10000},
    //             aptre[NUMVAR]={10000},
    //             asub[NUMANZ];
    // //系数的值
    // double      aval[NUMANZ];
    
    if(row_index==NUMCON){
        cout<<" row_index == NUMCON"<<endl;
    }
    else{
        cout<<" row_index != NUMCON"<<endl;
    }

    

    
    //代价函数矩阵计算
    Eigen::VectorXd w3(7),w4(7),w5(7),w6(7);
    w3<<-120,360,-360,120,0,0,0;
    w4<<360,-1440,2160,-1440,360,0,0;
    w5<<-360,1800,-3600,3600,-1800,360,0;
    w6<<120,-720,1800,-2400,1800,-720,120;
    Eigen::MatrixXd matrixQ(7,7);
    matrixQ=    w3*w3.transpose()   +w3*w4.transpose()/2+w3*w5.transpose()/3+w3*w6.transpose()/4+
                w4*w3.transpose()/2 +w4*w4.transpose()/3+w4*w5.transpose()/4+w4*w6.transpose()/5+
                w5*w3.transpose()/3 +w5*w4.transpose()/4+w5*w5.transpose()/5+w5*w6.transpose()/6+
                w6*w3.transpose()/4 +w6*w4.transpose()/5+w6*w5.transpose()/6+w6*w6.transpose()/7    ;
    matrixQ=2*matrixQ;
    for(int row=0;row<7;row++)
    {
        for(int col=0;col<=row;col++)
        {
            NUMQNZ++;
        }
    }
    NUMQNZ=NUMQNZ*3*traj_num;
    //代价函数矩阵Q
    MSKint32t   qsubi[NUMQNZ],
                qsubj[NUMQNZ];
    double      qval[NUMQNZ];
    int matrixQ_nz=0;
    for(int traj_i=0;traj_i<traj_num;traj_i++)
    {
        for(int dim_i=0;dim_i<3;dim_i++)
        {
            for(int row=0;row<7;row++)
            {
                for(int col=0;col<=row;col++)
                {
                    qsubi[matrixQ_nz]=traj_i*21+dim_i*7+row;
                    qsubj[matrixQ_nz]=traj_i*21+dim_i*7+col;
                    qval[matrixQ_nz]=matrixQ(row,col)/pow(time_slots[traj_i],1);
                    matrixQ_nz++;
                }
            }
        }
    }
    if(matrixQ_nz == NUMQNZ){
        cout<<"matrixQ_nz == NUMQNZ"<<endl;
    }
    else{
        cout<<"matrixQ_nz != NUMQNZ"<<endl;
    }


    MSKint32t    j, i;
    double       xx[NUMVAR];
    MSKenv_t     env;
    MSKtask_t    task;
    
    /* Create the mosek environment. */
    r = MSK_makeenv(&env, NULL);

    if ( r == MSK_RES_OK )
    {
        /* Create the optimization task. */
        r = MSK_maketask(env, NUMCON, NUMVAR, &task);

        if ( r == MSK_RES_OK )
        {
        r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);

        /* Append 'NUMCON' empty constraints.
        The constraints will initially have no bounds. */
        if ( r == MSK_RES_OK )
            r = MSK_appendcons(task, NUMCON);

        /* Append 'NUMVAR' variables.
        The variables will initially be fixed at zero (x=0). */
        if ( r == MSK_RES_OK )
            r = MSK_appendvars(task, NUMVAR);

        /* Optionally add a constant term to the objective. */
        if ( r == MSK_RES_OK )
            r = MSK_putcfix(task, 0.0);
        for (j = 0; j < NUMVAR && r == MSK_RES_OK; ++j)
        {
            /* Set the linear term c_j in the objective.*/
            if (r == MSK_RES_OK)
            r = MSK_putcj(task, j, c[j]);
            


            /* Set the bounds on variable j.
            blx[j] <= x_j <= bux[j] */
            if (r == MSK_RES_OK)
            r = MSK_putvarbound(task,
                                j,           /* Index of variable.*/
                                bkx[j],      /* Bound key.*/
                                blx[j],      /* Numerical value of lower bound.*/
                                bux[j]);     /* Numerical value of upper bound.*/
            
            int k=NUMCON;
            if(k==0)
                continue;
            

            // /* Input column j of A */
            // if (r == MSK_RES_OK)
            // {
            //     if(aptre[j]!=100000)
            //         r = MSK_putacol(task,
            //                 j,                 /* Variable (column) index.*/
            //                 aptre[j] - aptrb[j], /* Number of non-zeros in column j.*/
            //                 asub + aptrb[j],   /* Pointer to row indexes of column j.*/
            //                 aval + aptrb[j]);  /* Pointer to Values of column j.*/
            // }
            

        }

        //约束的系数稀疏矩阵传入
        if (r == MSK_RES_OK)
        {
            for(i=0;i<NUMCON;i++)
            {
                for(j=0;j<NUMVAR;j++)
                {
                    double coeff=fabs(matrixA[i][j]);
                    if(coeff> 1e-15 )
                    {
                        //cout<<"i "<<i<<" j "<<j<<" matrix "<< matrixA[i][j]<<" "<<" blc "<<blc[i]<<" buc "<<buc[i]<<" bkc "<<bkc[i]<<endl;
                        if (r == MSK_RES_OK)
                        r = MSK_putaij(task,
                            i,j,
                            matrixA[i][j]);
                    }
                }
            }
        }
        //cout<<"MSK_BK_FX "<<MSK_BK_FX<<endl;
        //cout<<"matrixA"<<endl;
       
        

        /* Set the bounds on constraints.
            for i=1, ...,NUMCON : blc[i] <= constraint i <= buc[i] */
        for (i = 0; i < NUMCON && r == MSK_RES_OK; ++i)
            r = MSK_putconbound(task,
                                i,           /* Index of constraint.*/
                                bkc[i],      /* Bound key.*/
                                blc[i],      /* Numerical value of lower bound.*/
                                buc[i]);     /* Numerical value of upper bound.*/

        if ( r == MSK_RES_OK )
        {
            /*
            * The lower triangular part of the Q^o
            * matrix in the objective is specified.
            */

            // qsubi[0] = 0;   qsubj[0] = 0;  qval[0] = 8;
            // qsubi[1] = 1;   qsubj[1] = 1;  qval[1] = 10;
            // qsubi[2] = 1;   qsubj[2] = 0;  qval[2] = 2;
            

            /* Input the Q^o for the objective. */

            r = MSK_putqobj(task, NUMQNZ, qsubi, qsubj, qval);
        }

        //   if ( r == MSK_RES_OK )
        //   {
        //     /*
        //     * The lower triangular part of the Q^0
        //     * matrix in the first constraint is specified.
        //     This corresponds to adding the term
        //     - x_1^2 - x_2^2 - 0.1 x_3^2 + 0.2 x_1 x_3
        //     */

        //     qsubi[0] = 0;   qsubj[0] = 0;  qval[0] = -2.0;
        //     qsubi[1] = 1;   qsubj[1] = 1;  qval[1] = -2.0;
        //     qsubi[2] = 2;   qsubj[2] = 2;  qval[2] = -0.2;
        //     qsubi[3] = 2;   qsubj[3] = 0;  qval[3] = 0.2;

        //     /* Put Q^0 in constraint with index 0. */

        //     r = MSK_putqconk(task,
        //                      0,
        //                      4,
        //                      qsubi,
        //                      qsubj,
        //                      qval);
        //   }

        if ( r == MSK_RES_OK )
            r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE);

        if ( r == MSK_RES_OK )
        {
            MSKrescodee trmcode;

            /* Run optimizer */
            struct timeval tv;
            gettimeofday(&tv,NULL);//获取1970-1-1到现在的时间结果保存到tv中
            double t1=tv.tv_sec*1000000+tv.tv_usec;
            
            r = MSK_optimizetrm(task, &trmcode);
            gettimeofday(&tv,NULL);//获取1970-1-1到现在的时间结果保存到tv中
            cout<<"usec "<<tv.tv_sec*1000000+tv.tv_usec-t1<<endl;
            /* Print a summary containing information
            about the solution for debugging purposes*/
            MSK_solutionsummary (task, MSK_STREAM_LOG);

            if ( r == MSK_RES_OK )
            {
            MSKsolstae solsta;
            int j;

            MSK_getsolsta (task, MSK_SOL_ITR, &solsta);

            switch (solsta)
            {
                case MSK_SOL_STA_OPTIMAL:
                case MSK_SOL_STA_NEAR_OPTIMAL:
                MSK_getxx(task,
                            MSK_SOL_ITR,    /* Request the interior solution. */
                            xx);

                printf("Optimal primal solution\n");
                for (j = 0; j < NUMVAR; ++j)
                    printf("x[%d]: %e\n", j, xx[j]);

                for(j=0;j<traj_num;j++)
                {
                    for(i=0;i<jerk_n;i++)
                    {
                        traj[j].x_ctr_point[i]=xx[j*21+i]; 
                    }
                    for(i=jerk_n;i<2*jerk_n;i++)
                    {
                        traj[j].y_ctr_point[i-jerk_n]=xx[j*21+i]; 
                    }
                    for(i=2*jerk_n;i<3*jerk_n;i++)
                    {
                        traj[j].z_ctr_point[i-2*jerk_n]=xx[j*21+i]; 
                    }
                }
                flag=1;
                break;

                case MSK_SOL_STA_DUAL_INFEAS_CER:
                case MSK_SOL_STA_PRIM_INFEAS_CER:
                case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
                case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:
                printf("Primal or dual infeasibility certificate found.\n");
                break;

                case MSK_SOL_STA_UNKNOWN:
                printf("The status of the solution could not be determined.\n");
                break;

                default:
                printf("Other solution status.");
                break;
            }
            }
            else
            {
            printf("Error while optimizing.\n");
            }
        }

        if (r != MSK_RES_OK)
        {
            /* In case of an error print error code and description. */
            char symname[MSK_MAX_STR_LEN];
            char desc[MSK_MAX_STR_LEN];

            printf("An error occurred while optimizing.\n");
            MSK_getcodedesc (r,
                            symname,
                            desc);
            printf("Error %s - '%s'\n", symname, desc);
        }
        }

        MSK_deletetask(&task);
    }
    MSK_deleteenv(&env);
}