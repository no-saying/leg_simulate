/* v3 纯 PID 静态配平参考，单位 m，相对 SW_NOM。
 * 与限速后的配平参考、分段腿摆伺服共同验证的扰动恢复范围约
 * 0.057..0.180m；表格两端保留外推值，不代表端点已通过验收。 */
#ifndef LEG_TRIM_TABLE_H
#define LEG_TRIM_TABLE_H

#define LEG_TRIM_N 17

typedef struct { float L; float sw_dev; } LegTrimPoint;
static const LegTrimPoint leg_trim_table[LEG_TRIM_N] = {
  { 0.0450f, +0.019000f },
  { 0.0570f, +0.019000f },
  { 0.0620f, +0.036000f },
  { 0.0700f, +0.036000f },
  { 0.0900f, +0.041000f },
  { 0.1100f, +0.041000f },
  { 0.1200f, +0.038000f },
  { 0.1260f, +0.036500f },
  { 0.1310f, +0.036000f },
  { 0.1320f, +0.036000f },
  { 0.1360f, +0.035000f },
  { 0.1410f, +0.036700f },
  { 0.1460f, +0.038000f },
  { 0.1500f, +0.041000f },
  { 0.1650f, +0.041000f },
  { 0.1850f, +0.042000f },
  { 0.2000f, +0.042000f },
};

#endif /* LEG_TRIM_TABLE_H */
