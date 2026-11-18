/**
 * @file catch_ground_cube.hpp
 * @brief 抓地立方体机构（新框架版）—— 双3508同步升降（一正一反）+ 双气缸可配模式控制
 * @attention
 *  1. 本类不依赖 Subsystem，由 CatchGroundCubeSubsystem 适配层托管生命周期；
 *     update() 只在"应当运行"时被调用，onRun(false)/掉线时由适配层调 stopOutput()。
 *  2. 所有外部设置都通过 applySetting() 一个函数传入（CatchSetMsg），
 *     手动桥接和后续自动控制共用这个接口。
 *  3. 双电机同步策略沿用 Lift 模块：共用目标 + 位置环(100Hz) + 双速度环(1kHz)
 *     + 同步误差交叉补偿；误差超阈值自动封波（本类管同步故障，掉线由 Subsystem 管）。
 *  4. 气缸驱动模式可配（config.valve_mode）：脉冲自保持阀 / 电平保持阀，见 ValveControlMode。
 * @versioninfo s24: 迁移到新框架（rc-framework-with-hfsm / 2027RC-BR1）。
 *             电机 API 换 C620Motor（set_output_command + enable 门控），
 *             PID 换 PidController，CAN ID 宏区删除（id 在实例化时给 1~8）。
 *             s32: 新增抓取流程（startGrab：合爪→等待锁定→自动抬升防摩擦），
 *             新增 CGC_VALVE_CLAMPED 夹紧电平约定（放置层 put_ground_cube 共用）。
 */

#pragma once

#include "main.h"
#include "c620.hpp"
#include "pid_controller.hpp"

// =====================================================================
//  硬件配置区 —— 本工程（2027RC-BR1）main.h 已有这些 GPIO 宏，按实际接线改
//  宏前缀 CGC = Catch Ground Cube（抓地立方体机构）
// =====================================================================
// 阀脚接线已确认（s32）：两个气缸经 MOS 驱动，阀1=PG3(PUMP_PICK)、阀2=PG4(VALVE_CLAW)
// 【待确认】阀号↔夹爪（上/下）对应关系由气缸测试阶段A确定后回填注释
// MOS 驱动电平语义：IO 高电平 = MOS 导通 = 阀得电（与代码 setValve(x, true)=得电 一致）
#define CGC_VALVE1_PORT          PUMP_PICK_GPIO_Port          // 气缸1（PG3，MOS 驱动）在上面
#define CGC_VALVE1_PIN           PUMP_PICK_Pin
#define CGC_VALVE2_PORT          VALVE_CLAW_GPIO_Port         // 气缸2（PG4，MOS 驱动）在下面
#define CGC_VALVE2_PIN           VALVE_CLAW_Pin

// 【待确认】夹紧电平约定：true = 阀逻辑"开"(得电)时爪子夹紧方块。
// 抓取流程(startGrab)与放置流程(PutGroundCube)共用本约定；装车后若反了只改这一处。
inline constexpr bool CGC_VALVE_CLAMPED = true;
// =====================================================================

/**
 * @brief 气缸驱动模式（config.valve_mode 取值，init 前设置，运行中勿改）
 * 选哪种取决于电磁阀硬件：试车不确定时可先保持默认，两种方式接口完全一致
 */
enum ValveControlMode : uint8_t {
  VALVE_MODE_PULSE = 0,  // 脉冲式自保持阀：一个脉冲=开/关切换，IO 不可长期带电（默认）
  VALVE_MODE_LEVEL = 1,  // 电平保持阀：IO 高电平=开、低电平=关，可持续带电
};

/**
 * @brief 机构设置信息（统一设置入口的数据结构）
 * 手动桥接和后续自动控制都填充这个结构体，调用 CatchGroundCube::applySetting()
 */
typedef struct {
  // ---- 升降（持续型，每帧都可传） ----
  float lift_delta_mm;     // 本帧目标增量（mm），正升负降；不需要动时传 0
  bool  lift_absolute;     // true 时忽略 delta，用 lift_target_mm 绝对覆盖目标
  float lift_target_mm;    // 绝对目标高度（mm，相对行程零点）
  // ---- 气缸（开关型，仅动作那一帧传非0，其余帧传0） ----
  int8_t valve1_cmd;       // 0=不动, 1=开, -1=关
  int8_t valve2_cmd;       // 0=不动, 1=开, -1=关
} CatchSetMsg;

class CatchGroundCube {
public:
  // 构造即绑定电机（引用，不可能空），PID 注册名字便于 PID Viewer 调参
  explicit CatchGroundCube(C620Motor& left, C620Motor& right);

  // ---------- 电机句柄（公有：Subsystem 适配层要做在线检查） ----------
  C620Motor& left_motor_;
  C620Motor& right_motor_;

  // ---------- 可调参数（init 时生效，运行时也可直接改） ----------
  struct Config {
    // 【待填】以下机械参数装车后实测填入
    float mm_per_deg          = 1.0f;     // 输出端1°对应的位移(mm)，实测后改
    float ground_clearance_mm = 0.0f;     // 行程零点相对地面的高度(mm)
    float travel_max_mm       = 100.0f;   // 最大行程(mm)，实测后改
    // 【待填】减速比/电流上限：3508 默认 19、20000mA；init(reduction, max) 会同步到电机
    float motor_reduction     = 19.0f;
    float motor_max_current   = 20000.0f;
    bool  left_invert         = true;     // 左电机指令取反（true=一正一反）
    float sync_threshold_deg  = 5.0f;     // 同步误差报警阈值(度)【待填】
    float sync_comp_gain      = 0.1f;     // 同步误差交叉补偿增益【待填】
    // 【待确认】气缸驱动模式，试车后按实际阀型定
    uint8_t  valve_mode       = VALVE_MODE_LEVEL;// VALVE_MODE_PULSE=脉冲自保持阀，VALVE_MODE_LEVEL=电平保持阀
    uint16_t valve_pulse_ms   = 50;       // 电磁阀驱动脉宽(ms)，仅脉冲模式使用，常见50~100
    // PID 参数（默认沿用 Lift 模板，装机构后实测微调）【待填】
    struct PidParam { float kp, ki, kd, max_out, integral_limit; };
    PidParam position_pid = {0.15f,   0.0f,   0.0f,     5.0f,    0.0f};  // 位置环→速度目标(rad/s)
    PidParam speed_pid    = {4000.0f, 1000.0f, 0.0f, 100000.0f, 5000.0f}; // 速度环→电流(mA)
    // ---- 抓取流程参数（s32 新增） ----
    float    lift_after_grab_mm = 0.0f;  // 【待填】抓取后自动抬升量(mm)：保证方块离地不摩擦；
                                         //        0=关闭（完全兼容 s24 旧行为），装车后填 10~30 左右实测
    uint16_t grab_settle_ms     = 80;    // 【待填】合爪后等待锁定(ms)，应略大于阀脉宽(valve_pulse_ms)
  } config;

  // ---------- 状态量（供外部读取/调试） ----------
  float target_position_deg_ = 0.0f;  // 当前目标位置（度，行程相对量）
  float sync_error_deg_      = 0.0f;  // 同步误差（度）
  bool  sync_fault_          = false; // 同步误差超限封波标志
  bool  initialized_         = false; // 首帧初始化完成标志

  // ---------- 生命周期 ----------
  void init();        // PID 配置 + 阀 IO 安全初始态（由适配层 onPrepare 调用，每次上线一次）
  void update();      // 控制主循环：1kHz 调用（内部分频：位置环100Hz + 速度环1kHz）
  void stopOutput();  // 关闭全部动力输出（适配层 onRun(false) 调用；阀脉冲收尾仍继续）
  void valveTick();   // 阀脉冲倒计时（update 内每拍调用；失能期间由 stopOutput 调用保证收尾）
  void holdCurrentPosition();  // 把目标钉在当前位置（适配层 onRequestReset 调用）

  // ---------- 统一设置入口（手动/自动共用的唯一接口） ----------
  void applySetting(const CatchSetMsg &setting);

  // ---------- 气缸接口（两种模式语义一致：set=到目标状态，Pulse=强制翻转） ----------
  void setValve1(bool open);   // 电平模式:直接写电平；脉冲模式:状态不符才发一个脉冲
  void setValve2(bool open);
  bool getValve1() const;      // 返回程序记账的逻辑状态（电平模式下即IO电平）
  bool getValve2() const;
  void valve1Pulse();          // 脉冲模式:发一个脉冲切换开/关；电平模式:退化为状态翻转
  void valve2Pulse();
  void resetValveState(bool valve1_open, bool valve2_open);  // 上电后校准记账（电平模式同时把IO驱到对应电平）

  // ---------- 升降辅助接口 ----------
  void addLiftDelta(float delta_mm);          // 增量调目标
  void setLiftTargetMm(float target_mm);      // 绝对设目标（内部用，不返回限位信息）
  bool moveToHeightMm(float target_mm);       // 给定高度自动走位（推荐外部入口，带限位反馈）
  float getHeightMm() const;                  // 当前高度(mm)
  float getSyncErrorMm() const;               // 同步误差(mm)

  // ---------- 抓取流程（s32 新增：合爪 → 等待锁定 → 自动抬升防摩擦） ----------
  enum class GrabSelection : uint8_t {
    BOTH = 0,        // 双爪同时合（抓地立方体的常规用法）
    VALVE1_ONLY,     // 只合阀1对应的爪（预留：空中夹取/截断流程）
    VALVE2_ONLY,     // 只合阀2对应的爪
  };
  enum class GrabState : uint8_t {
    IDLE = 0,        // 空闲
    CLOSING,         // 已发合爪指令，等待锁定(grab_settle_ms)
    LIFTING,         // 正在执行抓取后自动抬升
    DONE,            // 流程完成（结果态，重新 startGrab/cancelGrab 后离开）
  };
  bool startGrab(GrabSelection sel = GrabSelection::BOTH);  // 启动抓取流程（立即返回，非阻塞）
  // 实验性功能（不一定能用，见 put_ground_cube.hpp 功能4 注释）：startGrab(VALVE1_ONLY/
  // VALVE2_ONLY) 可在任意高度单爪合拢，配合 moveToHeightMm 实现"夹走对方未正确放置的方块"；
  // 规则 7.2 禁止触碰对方"已正确放置"的塔基/中段（取消资格），使用前对照规则原文确认
  void cancelGrab();                       // 中断流程：阀与升降目标保持现状，回到 IDLE
  bool isGrabBusy() const;                 // true=CLOSING/LIFTING 进行中
  GrabState getGrabState() const { return grab_state_; }

private:
  PidController position_pid_;        // 平台位置环（共用目标 → 速度目标）
  PidController left_speed_pid_;      // 左电机速度环（速度 → 电流）
  PidController right_speed_pid_;     // 右电机速度环（速度 → 电流）
  uint8_t position_loop_prescaler_ = 0;  // 位置环分频计数（10:1 → 100Hz）
  float position_loop_output_ = 0.0f;    // 位置环输出（平台速度目标 rad/s）
  float left_speed_target_  = 0.0f;      // 左速度环目标
  float right_speed_target_ = 0.0f;      // 右速度环目标

  // 气缸驱动状态（脉冲模式用 remaining 做脉宽倒计时，电平模式恒为 0）
  bool     valve1_open_           = false;  // 气缸1 逻辑状态（程序记账）
  bool     valve2_open_           = false;
  uint16_t valve1_pulse_remaining_ms_ = 0;  // 气缸1 剩余脉宽（valveTick 1kHz 下 1 计数=1ms）
  uint16_t valve2_pulse_remaining_ms_ = 0;

  float clampTarget(float target_mm, float current_mm) const;
  float platformPositionDeg() const;  // 读反馈算平台位置（内部带临界区）

  // ---------- 抓取流程状态（s32 新增） ----------
  GrabState     grab_state_        = GrabState::IDLE;
  GrabSelection grab_selection_    = GrabSelection::BOTH;
  uint16_t      grab_timer_ms_     = 0;    // CLOSING 锁定等待倒计时
  bool          grab_lift_pending_ = false;// 锁定完成后是否还要执行自动抬升
  void stepGrabSequence();                 // 抓取流程步进（update() 内 1kHz 调用）
};
