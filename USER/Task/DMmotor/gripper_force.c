#include "gripper_force.h"
#include "dm_motor_drv.h"
#include <math.h>

/* ============================================================================
 * 夹爪力控（MIT 模式 + 软件外环）实现 —— 设计说明见 gripper_force.h 文件头。
 *
 * 数据流：
 *   0x21 帧 int8 ──► gripper_force_apply()
 *                        │ ① 按符号选方向（目标角 / 速率 / 力矩上限）
 *                        │ ② |cmd| 归一化 → 该方向的力矩上限(N·m)
 *                        │ ③ 软件斜坡：pos_set 按 速率×dt 逼近目标角
 *                        │ ④ 力反馈外环：|para.tor| > 上限 ⇒ 冻结推进（迟滞释放）
 *                        │ ⑤ 目标超前实际上限兜底（τ ≤ kp×lead）
 *                        ▼
 *                  motor[Motor7].ctrl.{pos_set,vel_set,kp_set,kd_set,tor_set}
 *                        ▼
 *                  dm_motor_ctrl_send() → 按 ctrl.mode(mit_mode) 派发 mit_ctrl()
 *                                       → CAN ID = 0x07 + MIT_MODE(0x000) = 0x007
 * ==========================================================================*/

/* ---------------- 可在线调参 ---------------- */
/* 开合速度(rad/s)。★ 2026-10-11 用户定：手操轨迹回放实测 ≤0.28 rad/s，
 *   用户确认该速度可用，要求取它的**两倍**为基准 ⇒ 0.56。 */
volatile float GF_SPEED_OPEN    = 0.56f;
volatile float GF_SPEED_CLOSE   = 0.56f;

/* ★★★ 目标角(rad) —— 基于【夹爪张开位置 = 电机零点】的约定
 *
 * 零点约定：用 `dm_set_zero.py`（发 `FF×7 + 0xFE`）把**夹爪张开位置**写成电机 0 点。
 *
 * ★ 2026-10-15 **换新夹爪机构**后重新标定（零力帧下人工掰到两端，工具
 *   `dm_grip_point.py`，81 帧中位数、极差 0.02°）：
 *       张开 = 0             （设零点前是 -0.010109 rad）
 *       全闭 = -2.064736     （新坐标系；旧夹爪是 -2.0939）
 *       ⇒ 机械行程 = 2.064736 rad = **118.30°**（旧夹爪 120.05°）
 *   方向：pos 减小 = 闭合；pos 增大 = 张开（与旧夹爪一致，符号没变）。
 *
 * 目标取"越过端点 0.30 rad"：
 *   目标要一直顶着机构，位置误差才恒定 ⇒ 才有恒定夹持力；
 *   把目标刚好设在端点则到位后误差=0、τ=0，夹不住东西。
 *
 * ★★ 拆装电机后怎么恢复 —— **分两种情况，别搞混**：
 *     ① 只拆装、机械行程没变：手动摆到【张开】位置 + 重跑 `dm_set_zero.py`
 *        ⇒ 下面两个常量**一个都不用改**（零点只是 EEPROM 偏置）。
 *     ② **换了夹爪机构**（行程变了）：必须重新量行程 —— 摆到全开/全闭各抓一次
 *        （`dm_grip_point.py COMx --label ...`），按 ±0.30 重填下面两个常量。
 *        ★ 2026-10-15 就是这种情况：旧值 -2.3939 是按 120.05° 算的，
 *          新机构 118.30° ⇒ 必须改，否则闭合会多顶 0.0292 rad（1.7°）。 */
volatile float GF_POS_OPEN_RAD  = 0.30f;     /* 张开：0 再往外 0.30 ⇒ +0.30 */
volatile float GF_POS_CLOSE_RAD = -2.3647f;  /* 全闭：-2.064736 再往外 0.30 ⇒ -2.364736 */
volatile float GF_POS_LIMIT_RAD = 12.0f;   /* 对齐 DM4310 的 PMAX=12.5，留 0.5 余量 */

/* 夹持力矩上限(N·m) —— ★ 2026-10-16 改为【两向对称 1.5】。
 *
 *   历史：2026-10-10 用户定的是"张开 1.0 / 闭合 1.5"（不对称）。
 *   改动理由（用户 2026-10-16 拍板）：
 *     ① PC 侧映射刚做成**对称窗口**（±15° 死区 → ±90° 满力），
 *        力矩上限再不对称会让两个方向的手感不一致；
 *     ② `GF_POS_LEAD_RAD(0.30) × GF_KP(5.0) = 1.5` 本来就是兜底上限，
 *        张开也开到 1.5 正好与它对齐（不会出现"上限高于兜底"的空档）。
 *   ⚠️ 代价：**张开方向的力比原来 +50%**（1.0 → 1.5）。 */
volatile float GF_TAU_OPEN_NM   = 1.5f;
volatile float GF_TAU_CLOSE_NM  = 1.5f;
volatile float GF_TAU_HARD_NM   = 2.0f;
/* ★ 力矩曲线的**起点**(N·m)：克服机构静摩擦所需的最小力。
 *
 * 为什么必须有（2026-10-11 台架实测）：外环用 `|τ| > τ上限` 判"顶住"，
 * 若 τ上限 低于机构静摩擦，电机一动就被判成"顶住"而冻结，
 * 表现为"给小指令完全不动"。物理上说得通：**小于静摩擦的力等于没给力**，
 * 所以曲线的起点本来就应该落在静摩擦上。
 *
 * ★★ 2026-10-16 **台架实测替换**（`dm_stiction.py`，纯力矩法：kp=0/kd=0.3，
 *    τ 从 0 按 0.01 递增，轴开始动的那个 τ 就是静摩擦）：
 *       张开方向 0.120 / 0.120 / 0.120 N·m   （三次一位不差）
 *       闭合方向 0.140 / 0.150 / 0.160 N·m
 *    ⇒ 最坏 0.160 ⇒ 取 **0.20**（最坏值 +25% 余量，圆整）。
 *    ⚠️ 旧值 0.25 是台架**估**的（原注释"空载 DM4310 实测 |τ|≈0.09"），
 *       实测证明它偏高 —— 会让曲线的低力段"起步过重"。
 *    ⚠️ 换夹爪机构 / 拆装后应重跑 `dm_stiction.py` 复测。 */
volatile float GF_TAU_MIN_NM    = 0.20f;

/* MIT 增益。kp 决定"顶到 τ 上限所需的位移"（Δ = τ/kp）：kp=5 ⇒ 1.5 N·m 需 0.3 rad。
 * 起点取 5.0 —— 比 J1~J6 的 0.5 硬，因为夹爪要抵抗被抓物的反力。 */
volatile float GF_KP            = 5.0f;
volatile float GF_KD            = 0.5f;
volatile float GF_KD_FREE       = 0.3f;

volatile float GF_TAU_RELEASE   = 0.8f;    /* 迟滞释放：掉到 80% 上限才解除冻结 */
volatile float GF_POS_LEAD_RAD  = 0.30f;   /* τ ≤ kp×0.30 = 1.5 N·m（kp=5 时）*/

/* ★★★ 安全闩：0 = 未标定 ⇒ 全部降级为零状态（不控开闭、不出力、可自由掰动，
 *   且**不报错**）；1 = 放行（详见 gripper_force_apply / start_home 的两处检查）。
 *
 * ★ 2026-10-13 置 1.0f（永久化）：夹爪目标角已按"张开=电机零点"的约定标定完毕。
 * ★ 2026-10-15 **换新夹爪机构 + 重设电机零点**后按上面的流程重新标定、并复核无误，
 *   因此仍保持 1.0f：GF_POS_OPEN_RAD = +0.30 / GF_POS_CLOSE_RAD = -2.3647
 *   （行程 2.064736 rad = 118.30°）。
 *   在此之前这个值一直是**在线写 1**（掉电即回 0）⇒ 上位机发什么都被静默降级，
 *   现场表现为"夹爪纹丝不动、无任何报错"。改成 1.0f 后不再依赖在线写。
 *
 * ⚠️ 什么时候要把它改回 0.0f：只要你重设了电机零点（重跑 dm_set_zero.py）或换了
 *   夹爪机构，`GF_POS_OPEN_RAD / GF_POS_CLOSE_RAD` 就与实测行程脱钩 ⇒ 必须先回到
 *   标定流程（改这两个常量），确认后再置 1.0f。**标定 ≠ 改零点**这两件事都会让它失效。 */
volatile float GF_POS_VALID     = 1.0f;

/* ---- 回零（homing）参数 ---- */
volatile uint32_t GF_HOME_STABLE_FRAMES = 50;     /* 1 kHz ⇒ 50 ms */
volatile uint32_t GF_HOME_TIMEOUT_MS    = 8000;   /* 兜底：绝不永久占用控制权 */

/* 斜坡 dt 上限（评审 P1-4）：原来 0.200s ⇒ 停顿后一步推进 0.56×0.2 = 0.112 rad(6.4°)
 * 有冲击。钳到 20ms ⇒ 单步 ≤0.011 rad，且仍能容忍 20ms 级的调度抖动。
 * ⚠️ 这是"钳位"不是"补偿"：真停顿时长 >20ms 时位置会走得比物理时间慢一点，
 *    但那正是我们要的（宁可慢，不要在恢复瞬间跳）。 */
#define GF_DT_MAX        0.020f
#define GF_SPEED_MAX     20.0f        /* ramp 速率上限，防止在线把参数改到离谱值 */

/* ---------------- 内部状态 ---------------- */
static uint8_t  s_pos_valid  = 0;      /* 0 = 斜坡目标尚未对齐到实测位置（首帧要做）*/
static uint8_t  s_force_hold = 0;      /* 1 = 力外环已冻结推进 */
static float    s_ramp_pos   = 0.0f;   /* 当前已放行到的位置(rad) */
static uint32_t s_ramp_tick  = 0;      /* 上次更新时刻(HAL tick) */
static float    s_pos_cmd    = 0.0f;   /* 最近下发的目标角(rad) */
static float    s_tau_cmd    = 0.0f;   /* 最近一次的方向力矩上限(N·m) */
static int8_t   s_dir        = 0;      /* +1 张开 / -1 闭合 / 0 零状态 */

/* ---- 回零状态 ---- */
static uint8_t  s_homing      = 0;     /* 1 = 正在回零（外部命令被覆盖） */
static uint32_t s_home_t0     = 0;     /* 回零起始 tick */
static uint32_t s_home_stable = 0;     /* 连续"到位征兆"帧数 */

/* ------------------------------------------------------------------------- */
static float gf_minf(float a, float b) { return (a < b) ? a : b; }
static float gf_maxf(float a, float b) { return (a > b) ? a : b; }
static float gf_clampf(float x, float lo, float hi) { return gf_maxf(lo, gf_minf(x, hi)); }

/* 斜坡推进：把 cur 朝 tgt 方向推进最多 max_step */
static float gf_ramp(float cur, float tgt, float max_step)
{
    float d = tgt - cur;
    if (d >  max_step) return cur + max_step;
    if (d < -max_step) return cur - max_step;
    return tgt;
}

/* ------------------------------------------------------------------------- */
void gripper_force_probe_send(hcan_t *hcan)
{
    motor_t *m = &motor[Motor7];

    /* 零刚度 / 零阻尼 / 零前馈 / vel_set=0 的 MIT 帧：
     *   τ = 0×Δ + 0×Δv + 0 = 0 ⇒ **物理上不可能产生运动**。
     * 它的唯一作用是让电机进入 MIT 模式并开始周期上报反馈，
     * 好让 `motor[Motor7].para.pos` 尽快从 0 变成真值。 */
    mit_ctrl(hcan, m, m->id, m->para.pos, 0.0f, 0.0f, 0.0f, 0.0f);
}

void gripper_force_init(hcan_t *hcan)
{
    motor_t *m = &motor[Motor7];
    (void)hcan;

    /* ★ MIT 是 DM4310 的出厂默认模式（RID10=1）⇒ 使能帧就是 0x007。
     *   ⚠️ 若该电机的 RID10 被改成 2/4，MIT 帧会被**静默忽略**（夹爪纹丝不动、无报错）。
     *   （PSI 版本曾试图靠"使能帧 ID = id+0x300"来切模式 —— 台架实测证明无效，
     *     模式完全由电机的 RID10 决定。） */
    m->ctrl.mode = mit_mode;

    /* 使能前先把控制量压到底：零刚度、零前馈 ⇒ 即使电气上立刻使能，
     * 电机也不会因为一条陈旧的 pos_set 朝某个方向冲。 */
    m->ctrl.kp_set  = 0.0f;
    m->ctrl.kd_set  = GF_KD_FREE;
    m->ctrl.tor_set = 0.0f;
    m->ctrl.vel_set = 0.0f;
    m->ctrl.pos_set = m->para.pos;

    /* ★★ 关键（评审 P1-2 修复）：**不在这里**对齐斜坡目标。
     *   此刻 para.pos 可能还是 0（CAN 反馈尚未到达），拿它当基准毫无意义，
     *   而一旦让 s_ramp_pos 从 0 起步，标定放行后夹爪就会先朝绝对 0 rad 冲一下。
     *   改为把"对齐"推迟到下一次 gripper_force_apply() —— 那时电机已使能、
     *   反馈已经流出，para.pos 才是真值。
     *   这条同时兜住了 `DMmotorTask_Entry:473` 那条"裸 enable、没有 probe/init"的路径。 */
    s_pos_valid  = 0;
    s_force_hold = 0;
    s_dir        = 0;
    s_tau_cmd    = 0.0f;
    s_pos_cmd    = m->para.pos;
    s_ramp_pos   = m->para.pos;
    s_ramp_tick  = HAL_GetTick();

    /* 清掉上一次的回零状态：需要回零时必须由外部**显式**调用
     * `gripper_force_start_home()`（避免"上一次没走完就在使能时又接着跑"）。 */
    s_homing      = 0;
    s_home_stable = 0;
}

void gripper_force_disable(hcan_t *hcan)
{
    motor_t *m = &motor[Motor7];

    /* ★ 评审 P1-3 修复：先把"零刚度零力矩 + 目标 = 当前位置"**真正下发**出去，
     *   再发失能帧。旧实现只改了结构体字段却没调 dm_motor_ctrl_send()，
     *   那个 0 从未到达电机 ⇒ 注释说的"先卸力"实际没发生。 */
    m->ctrl.kp_set  = 0.0f;
    m->ctrl.kd_set  = 0.0f;
    m->ctrl.tor_set = 0.0f;
    m->ctrl.vel_set = 0.0f;
    m->ctrl.pos_set = m->para.pos;
    dm_motor_ctrl_send(hcan, m);

    dm_motor_disable(hcan, m);    /* 内部会 dm_motor_clear_para() 把 ctrl 清零 */

    s_force_hold = 0;
    s_dir        = 0;
    s_tau_cmd    = 0.0f;
    /* 下次使能后由 apply() 重新对齐（并再次走一遍"等反馈到了再定基准"的流程）*/
    s_pos_valid  = 0;
    s_ramp_tick  = HAL_GetTick();

    /* 失能即取消回零（失能状态下回零毫无意义）。 */
    s_homing      = 0;
    s_home_stable = 0;
}

/* ------------------------------------------------------------------------- */
void gripper_force_start_home(hcan_t *hcan)
{
    motor_t *m = &motor[Motor7];
    (void)hcan;

    /* ★ 未标定（安全闩为 0）时**拒绝启动回零** —— 目标角不可信，
     *   回零同样会朝着错误的方向冲。这是刻意的一致行为，不是遗漏。 */
    if (!(GF_POS_VALID > 0.5f))
    {
        s_homing = 0;
        return;
    }

    s_homing      = 1;
    s_home_t0     = HAL_GetTick();
    s_home_stable = 0;

    /* 清掉可能残留的力外环冻结，并把斜坡从**当前位置**起步：
     *   否则若上一次停在闭合方向，ramp 会滞留在旧方向的远端（那正是
     *   2026-10-11 台架抓到的"方向反转死锁"的成因）。 */
    s_force_hold = 0;
    s_ramp_pos   = m->para.pos;
    s_dir        = 0;        /* 强制走"方向变化"分支，下一帧再把 ramp 钉一次 */
}

int8_t gripper_force_is_homing(void) { return (int8_t)s_homing; }

/* ------------------------------------------------------------------------- */
void gripper_force_apply(hcan_t *hcan, int8_t cmd)
{
    motor_t *m = &motor[Motor7];

    /* ★★★ 安全闩（见 GF_POS_VALID 说明）：目标角没标定前，任何命令都降级成零状态。
     *   这是"上电不会自己冲"的最后一道保证，删掉它之前先读完那段注释。 */
    if (!(GF_POS_VALID > 0.5f))
    {
        cmd      = 0;
        s_homing = 0;   /* 未标定 ⇒ 回零同样被取消（目标角不可信，回零也是危险动作） */
    }

    /* ---- 0. 首帧对齐（评审 P1-2 修复）----
     *   到这里电机已经使能、反馈已经流出 ⇒ para.pos 可信。
     *   把斜坡目标钉在实际位置上，后续的推进都从这个真值出发，
     *   不会出现"从 0 rad 起步、先朝绝对 0 冲一下"。 */
    if (!s_pos_valid)
    {
        s_ramp_pos  = m->para.pos;
        s_pos_cmd   = m->para.pos;
        s_ramp_tick = HAL_GetTick();
        s_pos_valid = 1;
    }

    /* ---- 1. 斜坡的时间基准：用真实时间差，**不按调用次数** ----
     *   本项目已踩过"按次数算窗口"的坑：主循环周期一改，斜率就会静默变一个数量级。
     *   （注：只有在 User_defined / PC / MCU 某一档下才会调用本函数，档位切换时
     *     调用会间断 ⇒ 靠 HAL tick 算 dt 才正确，所以下面还要钳位。） */
    uint32_t now = HAL_GetTick();
    float dt = (float)(uint32_t)(now - s_ramp_tick) * 0.001f;   /* s */
    s_ramp_tick = now;
    if (!(dt > 0.0f)) dt = 0.001f;        /* 同 tick 内重复调用 */
    if (dt > GF_DT_MAX) dt = GF_DT_MAX;   /* P1-4：长停顿后不放大单步位移 */

    /* ---- 1.5 回零覆盖（2026-10-11 新增）----
     *   回零期间**忽略外部命令**，一律按"张开"驱动，直到判定到位 / 超时。
     *   注意它是**复用**下面整条链路（选方向 → 算目标角 → 斜坡限速 → 力外环限力
     *   → lead 兜底），所以回零的动作特性与正常张开**完全一致**，不存在第二套参数。
     *   （用户 2026-10-11 选定：回零优先，外部命令不能打断。） */
    if (s_homing)
    {
        const float home_tgt = gf_clampf(GF_POS_OPEN_RAD, -GF_POS_LIMIT_RAD, GF_POS_LIMIT_RAD);

        /* "到位征兆"两个，任一连续满足 GF_HOME_STABLE_FRAMES 帧即判到位：
         *   ① s_force_hold          ⇒ 力外环已判定"顶住"（有机构、真的顶到端点）
         *   ② ramp 已推到目标角     ⇒ 空载 / 没装限位机构时也成立
         * 用"连续"而不是"瞬时"：迟滞释放会让 hold 短暂掉一帧，瞬时判据会误判；
         * 两者都不满足时计数器清零，避免零散抖动累积成假到位。 */
        if (s_force_hold || (s_ramp_pos >= home_tgt - 1e-4f))
        {
            if (s_home_stable < GF_HOME_STABLE_FRAMES) s_home_stable++;
        }
        else
        {
            s_home_stable = 0;
        }

        if (s_home_stable >= GF_HOME_STABLE_FRAMES ||
            (uint32_t)(HAL_GetTick() - s_home_t0) > GF_HOME_TIMEOUT_MS)
        {
            s_homing      = 0;      /* 到位 / 超时 ⇒ 立刻交还控制权 */
            s_home_stable = 0;
        }
        else
        {
            cmd = (int8_t)100;      /* 回零优先：覆盖拨杆 / PC / MCU 的命令 */
        }
    }

    if (cmd == 0)
    {
        /* ---------- 零状态：不控开闭、也不出力 ----------
         * kp=0、tor=0 ⇒ 位置环无刚度、无前馈；只留 GF_KD_FREE 的阻尼
         * ⇒ 人可自由掰动夹爪，但不会像 kd=0 那样发抖（手册 3.1）。
         * 注意语义：这是**彻底不出力**，不是"锁在当前位置"——
         * 夹住的东西只靠机构自锁保持。这是用户明确要的语义。 */
        m->ctrl.pos_set = m->para.pos;
        m->ctrl.vel_set = 0.0f;
        m->ctrl.kp_set  = 0.0f;
        m->ctrl.kd_set  = GF_KD_FREE;
        m->ctrl.tor_set = 0.0f;

        s_ramp_pos   = m->para.pos;   /* 斜坡同步到当前位置 ⇒ 离开零状态时不跳变 */
        s_force_hold = 0;
        s_dir        = 0;
        s_pos_cmd    = m->para.pos;
        s_tau_cmd    = 0.0f;
    }
    else
    {
        const uint8_t opening = (cmd > 0);
        const float   u       = gf_minf(fabsf((float)cmd) * 0.01f, 1.0f);   /* 0..1 */

        /* ★★ 方向反转 ⇒ ① 清冻结 ② **把斜坡重新钉到实测位置**（2026-10-11 台架实测发现）。
         *
         *   只清冻结**不够**：反转瞬间 `s_ramp_pos` 还停在【旧方向的目标】上
         *   （闭合顶到位时 ramp ≈ pos − GF_POS_LEAD_RAD，因为 lead 兜底把它拉到那儿），
         *   此时测得 τ 仍是**反向的大值**（台架实测 −1.04 N·m），同一个函数里下面
         *   那条冻结判据会立刻重新冻上 ⇒ ramp 永不推进 ⇒ **死锁在旧方向**。
         *
         *   ⚠️ 这不是理论风险：**拨杆从"闭合"直接拨到"张开"必然踩到** ——
         *      中间没有 cmd==0 帧来把斜坡同步到当前位置。
         *
         *   把 ramp 钉到实测位置后 Δ=0 ⇒ τ=0 ⇒ 不会冻结，新方向从当前位置干净起步。
         *   （只在"方向变化"的那一帧执行，同向连续运动中不影响已推进的进度。） */
        if (opening != (s_dir > 0))
        {
            s_force_hold = 0;
            s_ramp_pos   = m->para.pos;
        }

        /* ① 方向 + 目标角（符号即方向）—— 双侧都受 GF_POS_LIMIT_RAD 保护 */
        float pos_tgt = opening ? GF_POS_OPEN_RAD : GF_POS_CLOSE_RAD;
        pos_tgt = gf_clampf(pos_tgt, -GF_POS_LIMIT_RAD, GF_POS_LIMIT_RAD);

        /* ② 推进速率 = 该方向的开合速度 */
        float vel = opening ? GF_SPEED_OPEN : GF_SPEED_CLOSE;
        vel = gf_clampf(vel, 0.0f, GF_SPEED_MAX);

        /* ③ 该方向的力矩上限 —— ★ 2026-10-16 改为【从静摩擦下限起步的线性映射】
         *
         *   旧写法 `max(u × τ_max, GF_TAU_MIN_NM)` 有个副作用：低端被"地板"压平 ——
         *   张开方向 `u < 0.25` 时（对应 PC 侧 15°~33.75°，占斜坡的 1/4）力矩恒为
         *   0.25 N·m，也就是**斜坡前四分之一没有手感差异**。
         *   而 PC 侧（`frame.py`）现在是"±15° 死区 + 之后**线性**到 90° 满力"
         *   ⇒ 这里必须同样线性，否则"线性"只在平段之后才成立。
         *
         *   新写法把下限当【起点】而不是【地板】：
         *       u = 0（刚出死区） ⇒ GF_TAU_MIN_NM（刚好能启动机构）
         *       u = 1（到软限位） ⇒ 该方向满力
         *   物理上也说得通：小于静摩擦的力等于没给力，曲线的起点本来就是静摩擦。
         *
         *   ⚠️ 判据：GF_TAU_MIN_NM 是用台架实测的**机构静摩擦**填的（`dm_stiction.py`），
         *      不是拍脑袋。若它大于该方向满力，曲线会反向 —— 下面 gf_clampf 兜住 [0, hard]。 */
        const float tau_max_dir = opening ? GF_TAU_OPEN_NM : GF_TAU_CLOSE_NM;
        float tau_lim = GF_TAU_MIN_NM + u * (tau_max_dir - GF_TAU_MIN_NM);
        tau_lim = gf_clampf(tau_lim, 0.0f, GF_TAU_HARD_NM);

        /* ---- ④ 力反馈外环：顶到力就冻结目标推进 ----
         *   τ_meas 是驱动器自己算的力矩（dm_motor_fbdata 解出的 para.tor）。
         *   松开物体、或还没接触到时 |τ| 很小 ⇒ 正常推进；
         *   顶住后位置误差使 τ 随斜坡增长，一旦越过 τ_lim 就**停止推进**。
         *   因为 pos_set 冻住了，Δ 不再增大 ⇒ τ 稳定在 τ_lim 附近 = 恒定夹持力。
         *   迟滞（GF_TAU_RELEASE）避免在阈值上下反复通断。
         *   ★ 先判断、后推进（而不是"先走一步再看"）：否则从"顶住"到"冻住"之间
         *     还会多走一帧（vel×dt ≈ 0.011 rad @1kHz），夹持末端有台阶。
         *
         *   ★★ 判据必须**带方向**（2026-10-11 台架实测发现）：
         *     "顶住" = 电机朝【命令方向】施力却被机构挡住 ⇒ τ 的符号应与命令方向一致。
         *     若 τ 与命令方向**相反**（例如张开命令下测到 −1.04 N·m），那含义是
         *     "还没走过去/正被反向推"，**不是顶住** ⇒ 绝不能冻结，否则会锁死在错误方向
         *     （台架实测：张开阶段第 0 帧就被反向 τ 冻死，7 s 只退了 0.054 rad）。
         *     => 用 `tor × dir` 这个**带符号力矩**做判据，而不是 |tor|。 */
        const float tau_signed = m->para.tor * (opening ? 1.0f : -1.0f);
        if (!s_force_hold)
        {
            if (tau_lim > 0.001f && tau_signed > tau_lim)
            {
                s_force_hold = 1;      /* 上一拍已经顶到力 ⇒ 本拍原地不动 */
            }
            else
            {
                s_ramp_pos = gf_ramp(s_ramp_pos, pos_tgt, vel * dt);
            }
        }
        else
        {
            if (tau_signed < tau_lim * GF_TAU_RELEASE) s_force_hold = 0;
        }

        /* ---- ⑤ 目标超前实际位置的兜底限幅 ----
         *   τ ≈ kp × (pos_set − pos) ⇒ 把超前量卡在 GF_POS_LEAD_RAD 之内，
         *   静态力矩就被 kp×lead 压住，**即使 kp 被在线调大也不会失控**。
         *   同时保证"位置反馈卡死/丢失"时目标不会无限远离实际位置。 */
        float lead = s_ramp_pos - m->para.pos;
        if (lead >  GF_POS_LEAD_RAD) s_ramp_pos = m->para.pos + GF_POS_LEAD_RAD;
        if (lead < -GF_POS_LEAD_RAD) s_ramp_pos = m->para.pos - GF_POS_LEAD_RAD;

        m->ctrl.pos_set = s_ramp_pos;
        m->ctrl.vel_set = 0.0f;          /* MIT 的 vel 是**阻尼项目标速度**，给 0 = 纯阻尼 */
        m->ctrl.kp_set  = GF_KP;
        m->ctrl.kd_set  = GF_KD;
        m->ctrl.tor_set = 0.0f;          /* 不用前馈：力矩完全由位置环 + 外环决定 */

        s_dir     = opening ? (int8_t)1 : (int8_t)-1;
        s_pos_cmd = s_ramp_pos;
        s_tau_cmd = tau_lim;
    }

    /* 按 ctrl.mode(=mit_mode) 派发 ⇒ mit_ctrl(hcan, m, 0x07, pos, vel, kp, kd, tor) */
    dm_motor_ctrl_send(hcan, m);
}

/* ------------------------------------------------------------------------- */
float  gripper_force_get_tau_cmd(void)  { return s_tau_cmd; }
float  gripper_force_get_pos_cmd(void)  { return s_pos_cmd; }
int8_t gripper_force_get_dir(void)      { return s_dir; }
int8_t gripper_force_is_holding(void)   { return (int8_t)s_force_hold; }
