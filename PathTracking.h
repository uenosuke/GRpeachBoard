#ifndef PATHTRACKING_h
#define PATHTRACKING_h

#include <Arduino.h>
#include <math.h>
#include "PIDclass.h"
#include "Filter.h"
#include "define.h"

#define PATHNUM 50
#define POINTNUM 100

#define FOLLOW_TANGENT  ( 0 )
#define FOLLOW_COMMAND  ( 1 )
//#define FOLLOW_ICHIPID  ( 2 )
#define POSITION_PID    ( 2 )

#define MODE_START  ( 0 )
#define MODE_NORMAL ( 1 )
#define MODE_STOP   ( 2 )
#define MODE_START_STOP   ( 3 )

class PathTracking{
public:
    /*********** 変数宣言 ***********/
    double Px[POINTNUM], Py[POINTNUM];
    double refangle[PATHNUM], refvel[PATHNUM];
    int acc_mode[PATHNUM], acc_count[PATHNUM];
    double dec_tbe[PATHNUM];

    // ベジエ曲線関連
    double Ax[ PATHNUM ];
    double Bx[ PATHNUM ];
    double Cx[ PATHNUM ];
    double Dx[ PATHNUM ];

    double Ay[ PATHNUM ];
    double By[ PATHNUM ];
    double Cy[ PATHNUM ];
    double Dy[ PATHNUM ];

    // 内積関連
    double a_be[ PATHNUM ];
    double b_be[ PATHNUM ];
    double c_be[ PATHNUM ];
    double d_be[ PATHNUM ];
    double e_be[ PATHNUM ];
    double f_be[ PATHNUM ];
    double d_be_[ PATHNUM ];
    double e_be_[ PATHNUM ];
    double f_be_[ PATHNUM ];

    double onx, ony;
    double angle, dist;
    double preAngle = 1.5708;
    double t_be, pre_t_be;
    double dist2goal;
    double epsilon;

    double refVx, refVy, refVz;
    double refKakudo;
    double tmpPx, tmpPy;

    /*********** 関数宣言 ***********/
    PathTracking(int xmode);
    // tを求めるための方程式
    double func(int p, double t);
    double dfunc(int p, double t);
    // tにおけるベジエ曲線の座標を求める関数
    double bezier_x(int p, double t);
    double bezier_y(int p, double t);
    // ベジエ曲線式の1階微分
    double dbezier_x(int p, double t);
    double dbezier_y(int p, double t);

    void initSettings();

    void calcRefpoint();
    int calcRefvel();
    
    void incrPathnum(double conv_length, double conv_tnum);
    void setConvPara(double conv_length, double conv_tnum);
    int getPathNum();
    void setPathNum(int);

    void setMode(int);
    int getMode();

    void setMaxPathnum(int);

    void setPosiPIDxPara(float xKp, float xKi, float xKd);
    void setPosiPIDyPara(float xKp, float xKi, float xKd);
    void setPosiPIDzPara(float xKp, float xKi, float xKd);
    void setYokozurePIDPara(float xKp, float xKi, float xKd);
    void setKakudoPIDPara(float xKp, float xKi, float xKd);
    void kakudoPIDinit();
    void setRefKakudo();

    double getRefVper();
    double getRefVrot();

private:
    int path_num;
    int mode;
    int max_pathnum;

    double conv_length;
    double conv_tnum;

    bool mode_changed;
    bool init_done;

    double tan, per, rot;
};

#endif