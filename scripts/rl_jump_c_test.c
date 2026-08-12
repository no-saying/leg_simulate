/* rl_jump_c_test.c - C 侧前向推理独立测试程序
 *
 * 读取 obs 文件(每行 RL_JUMP_N_IN 个 float), 调用 rl_jump_net_forward
 * 逐行推理, 输出到 out 文件(每行 RL_JUMP_N_OUT 个 float),
 * 供 scripts/verify_rl_jump_c.py 对比。
 *
 * 编译: gcc -O2 -I../control/controllers/rl_jump \
 *           rl_jump_c_test.c ../control/controllers/rl_jump/rl_jump_net.c -lm
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rl_jump_net.h"

/* 前向推理原型(实现在 rl_jump_net.c, 避免引入控制器头/传感器依赖) */
void rl_jump_net_forward(const float *in, float *out);

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: %s obs.txt out.txt\n", argv[0]);
        return 1;
    }
    FILE *fin = fopen(argv[1], "r");
    FILE *fout = fopen(argv[2], "w");
    if (!fin || !fout) {
        fprintf(stderr, "cannot open obs/out file\n");
        return 1;
    }
    char line[1024];
    int n = 0;
    while (fgets(line, sizeof(line), fin)) {
        float obs[RL_JUMP_N_IN];
        int got = 0;
        char *tok = strtok(line, " \t\n");
        while (tok && got < RL_JUMP_N_IN) {
            obs[got++] = (float)atof(tok);
            tok = strtok(NULL, " \t\n");
        }
        if (got != RL_JUMP_N_IN) {
            fprintf(stderr, "bad obs line %d (got %d/%d fields)\n",
                    n + 1, got, RL_JUMP_N_IN);
            return 1;
        }
        float out[RL_JUMP_N_OUT];
        rl_jump_net_forward(obs, out);
        for (int k = 0; k < RL_JUMP_N_OUT; k++)
            fprintf(fout, "%.8e%c", out[k], (k + 1 < RL_JUMP_N_OUT) ? ' ' : '\n');
        n++;
    }
    fclose(fin);
    fclose(fout);
    printf("C forward done: %d samples\n", n);
    return 0;
}
