#ifndef __GRIPPER_FORCE_H__
#define __GRIPPER_FORCE_H__

/* ============================================================================
 * 夹爪力控模块（Motor7 = DM4310）—— **MIT 模式 + 软件外环**
 *
 * ── 为什么从 PSI 改回 MIT（2026-10-11 台架实测结论，别再走回头路）──
 *   这台 DM4310（SubVer 003）的 PVT/PSI 环路【发散】：
 *     ① RID10=1(MIT) + 使能帧 0x307 → 纹丝不动（**模式由 RID10 决定，帧 ID 无影响**）
 *     ② RID10=4(PVT) + 目标=当前位置（理论零误差）→ 位置瞬跳十几 rad、vel 冲到 +3.85 rad/s
 *     ③ RID10=1 的 MIT、目标 −12.105 → **稳定收敛**到 −12.073，τ 仅 −0.10 N·m ✅
 *   ⇒ **MIT 是实测唯一可靠模式**；而且它是 DM4310 的**出厂默认（RID10=1）**，
 *     不需要用达妙助手改 EEPROM —— 这是它相对 PSI 的第二个好处。
 *
 *   ⚠️ 代价：MIT 帧没有"独立的速度给定 / 力矩上限"入口（`vel` 是阻尼项目标速度、
 *     `tor` 是前馈力矩，都不是限幅）。两个需求改由**软件外环**实现：
 *
 *     【定速开合】 软件斜坡：每次把发给驱动器的 `pos_set` 按 `speed × dt` 推进
 *                 （`gf_ramp`）。这是**唯一有效的限速手段** —— 台架实测 PVT 的
 *                 vel 字段限不住速度（给 0.5 rad/s 却跑出 ~7 rad/s）。
 *
 *     【夹持力上限】外环冻结：读 `motor[Motor7].para.tor`（驱动器自己算出的力矩），
 *                 |τ| 超过该方向的 τ 上限 ⇒ **冻结 pos_set 的推进**。
 *                 顶住物体后位置误差使 τ≈kp·Δ 随斜坡增长，到上限即停
 *                 ⇒ "顶到力就停"，不会一直蠕变、也不会无限加力。
 *                 释放用迟滞（`GF_TAU_RELEASE`），避免在阈值附近反复通断。
 *
 *     【零状态】   kp=0、kd=`GF_KD_FREE`、tor=0 ⇒ τ = −kd·v，只剩一点阻尼，
 *                 人可自由掰动夹爪。这是 cmd==0 的语义。
 *
 * MIT 控制律（达妙手册 3.1）：
 *     τ = kp·(pos_set − pos) + kd·(vel_set − vel) + tor_set
 *
 * 上位机语义（0x21 里夹爪只占 1 个字节 int8；**协议长度零改动，LEN 仍 27**）：
 *     cmd = 0            → 零状态（不控开闭、也不出力）
 *     cmd ∈ (+1..+100]   → 张开，力矩上限 = |cmd|/100 × GF_TAU_OPEN_NM
 *     cmd ∈ [-100..-1)   → 闭合，力矩上限 = |cmd|/100 × GF_TAU_CLOSE_NM
 *
 * ⚠️ 上实机前必读：
 *   1. 夹爪电机的 `RID10` 必须是 **1(MIT)**。若被改成 2/4，MIT 帧会被**静默忽略**
 *      （现象：夹爪纹丝不动、无任何报错）。用达妙助手读一遍确认。
 *   2. `GF_POS_OPEN_RAD / GF_POS_CLOSE_RAD` 是**绝对角**，出厂占位值 ±3.0 rad
 *      与你的夹爪行程毫无关系 ⇒ 标定完再把 `GF_POS_VALID` 置 1.0。
 * ==========================================================================*/

#include "dm_motor_ctrl.h"   /* motor_t / extern motor[num] */

/* ---- 生命周期 ---- */
/** ★★ 上电时序第一步（在电机【失能】时调用）：发一帧**零刚度零力矩**的 MIT 帧。
 *
 *  目的只有一个 —— 让电机进入 MIT 模式并开始周期上报反馈，好让 `para.pos`
 *  尽快变成真值。帧内 kp/kd/tor/vel 全 0 ⇒ **不可能产生任何运动**。
 *
 *  ★ 对比旧 PSI 版：那里发的是 `pos = 0.0` 的 PSI 帧，在 PVT 下是一条
 *    "去绝对 0 rad"的**位置指令**（评审 P1-1），只是靠"此刻失能"才侥幸安全。
 *
 *  调用方式（见 DMmotor_task.c）：
 *      for (int i = 0; i < 3; i++) { gripper_force_probe_send(&hfdcan2); vTaskDelay(5); }
 *      gripper_force_init(&hfdcan2);
 *      dm_motor_enable(&hfdcan2, &motor[Motor7]); */
void gripper_force_probe_send(hcan_t *hcan);

/** 切到 MIT 模式 + 清零控制量。
 *  **必须在 dm_motor_enable(&motor[Motor7]) 之前调用**。
 *  它同时把"斜坡目标对齐"推迟到下一次 `gripper_force_apply()` —— 那时电机
 *  已使能、CAN 反馈已经流出，`para.pos` 才是可信的（修掉评审 P1-2）。 */
void gripper_force_init(hcan_t *hcan);

/** 失能：先**真正下发**一帧"零刚度零力矩"指令卸力，再发失能帧。
 *  （评审 P1-3：旧实现只改结构体字段却没 `ctrl_send`，那个 0 从未到达电机。） */
void gripper_force_disable(hcan_t *hcan);

/* ---- 运行 ---- */
/** 每周期调用一次（由 `DMcontrol_motor_7` 转发）。cmd 语义见文件头。 */
void gripper_force_apply(hcan_t *hcan, int8_t cmd);

/* ---- 回零（homing）---- */
/** 启动一次"回零"：把夹爪驱动到**张开零点**（目标角 = `GF_POS_OPEN_RAD`）。
 *
 *  ── 为什么需要它 ──
 *    上电 / 遥控器使能 / 档位切换这三个时刻，夹爪过去只做 `init`（把斜坡目标
 *    对齐到**当前位置**）⇒ **停在原地，位姿不确定**。而下游（遥操作映射、
 *    PC 数据采集、VLA 推理）都假定"夹爪从张开位开始"。
 *
 *  ── 实现 ──
 *    置一个 `s_homing` 标志，`gripper_force_apply()` 每帧检查；**未到位就强制按
 *    "张开"驱动**（等价于 cmd = +100）。这样它**完整复用**了已有的软件斜坡
 *    （限速 `GF_SPEED_OPEN`）与力外环（限力 `GF_TAU_OPEN_NM` + 顶到位冻结），
 *    不必另写一套运动逻辑，也就不会有第二套参数需要维护、不会再出现"改了固件
 *    忘了改这里"的漂移。
 *
 *  ── 结束条件（三者取先到）──
 *    · 力外环连续 `GF_HOME_STABLE_FRAMES` 帧处于"顶住" ⇒ 确实到了机械端点；
 *    · 斜坡已推进到目标角后同样连续若干帧            ⇒ 空载/无限位时也成立；
 *    · 超过 `GF_HOME_TIMEOUT_MS`                     ⇒ 兜底，**绝不永久占用控制权**。
 *    结束即把控制权交还外部命令（拨杆 / PC / MCU）。
 *
 *  ── 注意 ──
 *    ⚠️ 依赖 `GF_POS_VALID > 0.5`：未标定时安全闩会把回零**一并降级成零状态**
 *       （这是刻意的 —— 目标角不可信时，回零同样是危险动作）。
 *    ⚠️ 回零期间**外部命令被忽略**（回零优先，用户 2026-10-11 选定）。
 *    ⚠️ 应在 `gripper_force_init()` + `dm_motor_enable()` **之后**调用。 */
void gripper_force_start_home(hcan_t *hcan);

/** 1 = 正在回零（调试 / 状态上报用）。 */
int8_t gripper_force_is_homing(void);

/* ---- 观测（调试 / 状态上报）---- */
/** 当前方向允许的力矩上限(N·m)。零状态为 0。 */
float  gripper_force_get_tau_cmd(void);
/** 最近一次下发的目标角(rad)。 */
float  gripper_force_get_pos_cmd(void);
/** 上一拍方向：+1 张开 / −1 闭合 / 0 零状态。 */
int8_t gripper_force_get_dir(void);
/** 1 = 力外环已冻结推进（正顶着物体夹持）/ 0 = 正常推进。 */
int8_t gripper_force_is_holding(void);

/* ---- 可在线调参（volatile，调试器 / 上位机可直接改，不必重烧）---- */
/** 开合速度(rad/s) —— **软件斜坡的斜率**，也是唯一有效的限速手段。
 *  ★ 取值依据（2026-10-11）：用用户手操轨迹回放实测，电机实际跑 ≤0.28 rad/s，
 *    用户确认"那个速度可以"，要求把它的**两倍**作为基准运动速度 ⇒ 0.56 rad/s。
 *    （若你当时指的是脚本设定值 0.2 的两倍，这里改成 0.4 即可，只此一处。） */
extern volatile float GF_SPEED_OPEN;
extern volatile float GF_SPEED_CLOSE;

/** ★★★【上实机前必须标定】张开 / 闭合方向的**目标角(rad，单圈绝对)**。
 *  ±3.0 只是占位值。标定法：
 *    ① 夹爪停在机械全开位，读 `motor[Motor7].para.pos` → POS_OPEN
 *    ② 夹爪停在机械全闭位，读 `motor[Motor7].para.pos` → POS_CLOSE
 *    ③ 目标取"越过端点一点"，这样才会一直朝那个方向顶住、给出恒定夹持力：
 *       GF_POS_OPEN_RAD  = POS_OPEN  + 0.3（符号按实测方向）
 *       GF_POS_CLOSE_RAD = POS_CLOSE − 0.3 */
extern volatile float GF_POS_OPEN_RAD;
extern volatile float GF_POS_CLOSE_RAD;
/** 目标角绝对值上限(rad)，与 motor7 的 PMAX(12.5) 对齐、留 0.5 余量。 */
extern volatile float GF_POS_LIMIT_RAD;

/** 张开 / 闭合方向的**夹持力矩上限(N·m)**，随 |cmd| 归一化后使用。 */
extern volatile float GF_TAU_OPEN_NM;
extern volatile float GF_TAU_CLOSE_NM;
/** ★ **夹持力上限的下限(N·m)** —— 克服机构静摩擦所需的最小力。
 *  外环用 `|τ| > τ上限` 判"顶住"，而 τ上限 = `|cmd|/100 × τ_max`：
 *  当 |cmd| ≤ 10 时上限只有 0.06~0.15 N·m，**比电机自身静摩擦还小**
 *  （空载 DM4310 实测 |τ|≈0.09，真夹爪更大）⇒ 一动就被判"顶住"冻结，
 *  表现为"给小指令完全不动"（台架日志 t=9.0s / t=22.0s 都出现过）。
 *  物理上小于静摩擦的力等于没给力，所以这里给一个下限。
 *  调成 0 即恢复"|cmd| 线性映射"的原始行为。 */
extern volatile float GF_TAU_MIN_NM;
/** 硬上限(N·m)：任何情况下都不超过它（纵深防御）。 */
extern volatile float GF_TAU_HARD_NM;

/** MIT 位置刚度 kp（0~500）。越大越硬、到达 τ 上限所需的位移越小。
 *  夹持力 ≈ kp × Δ（Δ = 目标超前实际量）⇒ 它和 `GF_POS_LEAD_RAD` 共同决定
 *  最大静态夹持力，调参时两者一起看。 */
extern volatile float GF_KP;
/** MIT 阻尼 kd（0~5）。非零 cmd 时使用。 */
extern volatile float GF_KD;
/** 零状态（cmd==0）用的阻尼。**刻意不为 0** —— 手册 3.1 警告位置控制 kd=0 会抖。
 *  但也不宜大：这一档的语义是"不出力、可自由掰动"。 */
extern volatile float GF_KD_FREE;

/** 力外环的**迟滞释放**系数：|τ| 掉到 `τ上限 × 本值` 以下才解除冻结。
 *  取 1.0 会在阈值上反复通断；取太小则夹持保持不住。0.8 是稳妥起点。 */
extern volatile float GF_TAU_RELEASE;
/** 目标**超前实际位置**的允许上限(rad) —— 间接限力的兜底：
 *  即使 kp 被在线调大，静态力矩也被 `kp × 本值` 压住。
 *  也顺带保证"反馈断了 / 位置卡死"时目标不会无限远离实际。 */
extern volatile float GF_POS_LEAD_RAD;

/** ★★★【安全闩】0 = 未标定 ⇒ 所有命令降级成零状态（不控开闭、不出力、可自由掰动）。
 *  标定完 `GF_POS_OPEN_RAD / GF_POS_CLOSE_RAD` 后置 1.0 才放行。详见 gripper_force.c。
 *  ⚠️ 回零（`gripper_force_start_home`）同样受它管辖。 */
extern volatile float GF_POS_VALID;

/* ---- 回零参数（volatile，可在线调）---- */
/** 回零"到位"判据的**连续帧数**（1 kHz 主循环 ⇒ 50 帧 = 50 ms）。
 *  同时用于两个信号：力外环"顶住"、或斜坡已推到目标角。
 *  取 50 是为了容忍顶住瞬间的抖动（迟滞释放会让 hold 短暂掉一次）。 */
extern volatile uint32_t GF_HOME_STABLE_FRAMES;
/** 回零超时(ms) —— 兜底：无论什么原因（机构卡住 / 反馈丢失 / 目标角被在线改坏）
 *  都不能让回零**永久占用控制权**。按最坏情况估算：从闭合端点 −2.39 走到
 *  张开目标 +0.30 共 2.7 rad ÷ 0.56 rad/s ≈ 4.8 s，留一倍余量 ⇒ 8 s。 */
extern volatile uint32_t GF_HOME_TIMEOUT_MS;

#endif /* __GRIPPER_FORCE_H__ */
