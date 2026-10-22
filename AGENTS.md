# AGENTS.md — 本项目给 AI Agent 的入口

> 任何 agent 在本仓库动手前，**先读这两篇**（别通读 `docs/`，那是浪费 token）。

## 必读顺序

| # | 文档 | 何时读 |
|---|---|---|
| 1 | `docs/项目执行-上手与防错手册.md` | **动手前必读**：15 分钟检查清单、红线、代码地图、验证规矩、坑表、汇报模板、当前不要动的区域 |
| 2 | `docs/项目总规划-交接简报.md` | 需要"为什么这样做"时读 §3（物理意义）/§8（进度与风险）；规划类任务整篇读 |
| 3 | `docs/增量开发上手指南-给Agent.md` | 要写代码前查项目背景、技术栈、业务能力、协作约束 |

## 五条最容易被违反的红线

1. **依赖版本精确锁定**：Qt 6.7.3 / PCL 1.14.0 / VTK 9.3.0 —— 不得升级或降级。
2. **不改 `CMakeLists.txt`**：`GLOB ... CONFIGURE_DEPENDS` 会自动收集新源码；验证程序放 `analysis/`，不进 CMake。
3. **不做破坏性清理**：不删未跟踪文件、不重建 `.sln/.vcxproj`。
4. **线程分工**：计算类不依赖 Qt；VTK/Qt/查看器/点云表/撤销只在 GUI 线程；worker 用 `PostProgress/PostLog`，结果 `shared_ptr` 回传。
5. **禁止 `pkill -f "build/CloudForgeAnalyzer"`**（会杀掉自己所在的 shell）；用 `pkill -x CloudForgeAnalyze`。

## 工作循环

本机约 22 GiB 内存，曾因 16 并发编译触发 OOM。构建必须最多 2 并发，并使用 `analysis/run_guarded.sh` 限制任务内存为 8 GiB；不得同时启动多个构建。

```bash
# 改 → 构建 → 跑本功能的验证程序 → 与基线对照 → 汇报
export VCPKG_ROOT=/home/jamesyasr/dsh-workspace/vcpkg
analysis/run_guarded.sh taskset -c 0-3 cmake --build build --parallel 2

# 凹塘（1.7）
analysis/cylinder_patch/build_test.sh
./build/test_cyl_pothole PCDfiles/pothole_dent_form.pcd 1940 local=1 win=90 lthr=0.35 \
  minpts=60 ctol=5 axis=analysis/cylinder_patch/out_multipit/axis_form.txt   # 期望 [PITCOUNT] 4

# 焊前装配阶差/间隙（1.3/1.4）
analysis/weld_preparation/build_test.sh
OMP_NUM_THREADS=8 taskset -c 0-7 ./build/test_weld_prep --quick   # A/B 层约 1 分钟，期望 pass=57 fail=0
```

- 算法改动必须给出与基线的对照：**逐位一致，或逐条解释差异**（凹塘基线 `analysis/cylinder_patch/out_multipit/v9_final.txt`）。
- 动了 UI/可视化，要在桌面会话里实际点一遍并记录看到什么。
- 结论有变化或需甲方决策的事项，标注"**待确认**"，不要写成已完成。

## 验证边界（用户明确要求）

- 可以随时跑**本功能自己**的验证程序。
- **不要擅自跑其它功能的回归**；确有必要时先说明"改了哪个共享文件 → 担心影响哪个功能 → 建议跑哪个验证 → 预计耗时"，
  **征得用户同意后再跑**（共享文件：`Src/CloudForgeAnalyzer.cpp`、`CloudForgeAnalyzer.ui`、`Inc/CloudForgeAnalyzer.h`、`Inc/Measure/Measure.h`）。

## 客户可见文本规范

**软件里只输出测量结果**（用户明确要求）：报告、结果面板、日志行**不得暴露内部调试或设计思路**——
不写算法参数（栅格/邻域/搜索窗口/点距）、不写"口径说明/基准来源"，不写"为什么不判合格"这类方法论说明，
不写内部判据名（如"邻域支撑不足""平行退化""误差放大"）与诊断计数；失败时只给一句结果级原因
（例：`不可测: 沿轴向未找到近件边缘与远件边缘的对应点`）。进度提示只保留操作动作词。

- 术语：用 **局部基准 / 全局基准（到理想柱面距离）**，**禁止**出现"口径A/口径B"这类内部叫法。
- 报告精简：结论先行、逐坑一行（深度 | 椭圆长短轴 | 点数 | 位置 | 通过与否），必留"到理想柱面最大距离与位置"。
- 3D 标注：字高按场景自适应 `clamp(2.2% × 包围盒对角线, 6, 14) mm`；各坑标签抬 1.15 倍字高，
  "最大距离"标签抬 2.6 倍字高并加引线（两者常只差几毫米，同高必然叠字）。
