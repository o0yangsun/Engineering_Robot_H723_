#include "trajectory_time_synchronization.h"
#include <math.h>

/* 统一进度轨迹总位移固定为 1.0
 * 本模块只负责生成从 s=0 到 s=1 的归一化进度轨迹 */
#define TIME_SYNC_TOTAL_PROGRESS   (1.0f)

/* 小阈值：防止把过小的速度、加速度当成有效参数 */
#define TIME_SYNC_EPS              (1e-4f)

/* 限幅函数：把 x 限制在 [xmin, xmax] */
static float clampf_local(float x, float xmin, float xmax)
{
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

bool TrajTimeSync_Start(SpeedTimeSYNC_t *profile, float v_max, float a_max)
{
    float accel_time_candidate;
    float accel_end_progress_candidate;

    if ((profile == 0) || (v_max <= TIME_SYNC_EPS) || (a_max <= TIME_SYNC_EPS))
    {
        return false;
    }

    /* 初始化轨迹状态 */
    profile->progress_vel_max = v_max;
    profile->progress_acc_max = a_max;
    profile->current_time = 0.0f;
    profile->use_triangle_profile = 0;
    profile->running = 1;

    /* 假设可以跑到最大速度所需的最大时间 */
    accel_time_candidate = v_max / a_max;

    /* 对应加速段进度 */
    accel_end_progress_candidate = 0.5f * a_max * accel_time_candidate * accel_time_candidate;

    /* 能否形成标准梯形速度 */
    if ((2.0f * accel_end_progress_candidate) < TIME_SYNC_TOTAL_PROGRESS)
    {
        /* 梯形速度轨迹 */
        profile->use_triangle_profile = 0;
        profile->accel_time = accel_time_candidate;
        profile->accel_end_progress = accel_end_progress_candidate;
        profile->peak_progress_vel = v_max;
        profile->cruise_time =
                (TIME_SYNC_TOTAL_PROGRESS - 2.0f * profile->accel_end_progress) / profile->peak_progress_vel;
        profile->total_time = 2.0f * profile->accel_time + profile->cruise_time;
    }
    else
    {
        /* 三角速度轨迹 */
        profile->use_triangle_profile = 1;
        profile->accel_time = sqrtf(TIME_SYNC_TOTAL_PROGRESS / a_max);  //1=2*0.5*a*t?  -> t=sqrtf(1/a)
        profile->accel_end_progress =
                0.5f * a_max * profile->accel_time * profile->accel_time;
        profile->peak_progress_vel = a_max * profile->accel_time;
        profile->cruise_time = 0.0f;
        profile->total_time = 2.0f * profile->accel_time;
    }

    return true;
}

bool TrajTimeSync_Update(SpeedTimeSYNC_t *profile,
                       float dt,
                       float *progress,
                       float *progress_vel)
{
    float current_time;
    float decel_local_time;

    if ((profile == 0) || (progress == 0) || (progress_vel == 0) || (dt < 0.0f))
    {
        return false;
    }

    /* 轨迹已结束：持续输出终点 */
    if (profile->running == 0)
    {
        *progress = 1.0f;
        *progress_vel = 0.0f;
        return true;
    }

    current_time = profile->current_time;

    /* 起点 */
    if (current_time <= 0.0f)
    {
        *progress = 0.0f;
        *progress_vel = 0.0f;
    }
        /* 加速段 */
    else if (current_time < profile->accel_time)
    {
        *progress = 0.5f * profile->progress_acc_max * current_time * current_time;
        *progress_vel = profile->progress_acc_max * current_time;
    }
        /* 匀速段（仅梯形轨迹存在） */
    else if ((profile->use_triangle_profile == 0) &&
             (current_time < (profile->accel_time + profile->cruise_time)))
    {
        *progress = profile->accel_end_progress +
                    profile->peak_progress_vel * (current_time - profile->accel_time);

        *progress_vel = profile->peak_progress_vel;
    }
        /* 减速段 */
    else if (current_time < profile->total_time)
    {
        if (profile->use_triangle_profile)
        {
            decel_local_time = current_time - profile->accel_time;

            *progress = profile->accel_end_progress +
                        profile->peak_progress_vel * decel_local_time -
                        0.5f * profile->progress_acc_max * decel_local_time * decel_local_time;

            *progress_vel = profile->peak_progress_vel -
                            profile->progress_acc_max * decel_local_time;
        }
        else
        {
            decel_local_time = current_time - (profile->accel_time + profile->cruise_time);

            *progress = profile->accel_end_progress +
                        profile->peak_progress_vel * profile->cruise_time +
                        profile->peak_progress_vel * decel_local_time -
                        0.5f * profile->progress_acc_max * decel_local_time * decel_local_time;

            *progress_vel = profile->peak_progress_vel -
                            profile->progress_acc_max * decel_local_time;
        }
    }
        /* 完成段 */
    else
    {
        *progress = 1.0f;
        *progress_vel = 0.0f;
        profile->running = 0;
        return true;
    }

    *progress = clampf_local(*progress, 0.0f, 1.0f);
    if (*progress_vel < 0.0f)
    {
        *progress_vel = 0.0f;
    }

    /* 推进内部时间 */
    profile->current_time += dt;
    if (profile->current_time >= profile->total_time)
    {
        profile->current_time = profile->total_time;
    }

    return true;
}