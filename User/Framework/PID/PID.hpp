//
// Created by 42038 on 2026/8/15.
//

#ifndef RM_CAN_PID_HPP
#define RM_CAN_PID_HPP

#define PID_DEFAULT_PRECISION 0.0f//最小差距
#define PID_DEFAULT_ERRALL_MAX 3000000//最大误差
#define PID_DEFAULT_OUTPUT_MAX 10192 //输出限幅
#define PID_DEFAULT_OUTPUT_STEP_MAX 3192//输出步长限制?

typedef struct {
    float Kp1;
    float Ki1;
    float Kd1;

    float PID_Err_now;
    float PID_Err_last;
    float PID_Err_all;

    float PID_Out;
    float PID_lastout;
    float PID_Target;//目标输入
    float PID_lastTarget;
    float PID_Input;//反馈量

    float PID_Precision;//最小精度
    float PID_ErrAllMax;//积分限幅
    float PID_OutMax;//输出限幅
    float PID_OutStep;//输出步幅限制
} PID_t;

class PIDc {
private:
    void PID_Update(PID_t *WhichPID,float NowInput,float Target);
    float PID_Calculate(PID_t *WhichPID);
public:
    void PID_Init(PID_t *WhichPID,float Kp,float Ki,float Kd);
    float Pos_Spd_PID(PID_t *SpdParam,PID_t *PosParam,float Pos_Input,float Pos_Target,float Spd_Input);
    float SingleLoop_PID(PID_t *Which_PID, float Pos_Input, float Pos_Target);
    void PID_Clear();
    PID_t SpdParam;
    PID_t PosParam;
};

#endif //RM_CAN_PID_HPP