/*
 * controller.c - COD-2026 自平衡机器人仿真入口
 * ================================================================
 * 原 785 行单体文件已按功能拆分为:
 *   - app_internal.h   : 应用层共享结构/函数声明
 *   - app_sim.c        : 模型加载/复位/单步推进
 *   - app_keys.c       : 键盘状态与回调
 *   - app_headless.c   : 无界面测试模式
 *   - app_gui.c        : GLFW 窗口/相机/渲染/手动指令
 *
 * 本文件只保留 main: 加载模型后按命令行参数选择 GUI 或无界面模式。
 */
#include "app_internal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
    Simulation sim;
    if (!simulation_load(&sim, "model/v3/COD-2026RoboMaster-Balance-v3.xml"))
        return 1;

    int rc;
    if (argc > 1 && strcmp(argv[1], "--headless") == 0) {
        double duration = (argc > 2) ? atof(argv[2]) : 13.0;
        if (duration <= 0.0) duration = 13.0;
        rc = run_headless(&sim, duration);
    } else {
        rc = run_gui(&sim);
    }

    simulation_destroy(&sim);
    return rc;
}
