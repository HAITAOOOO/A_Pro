#ifndef __JY61P_H_
#define __JY61P_H_

#define AX 0x34

typedef struct
{

    float pitch; // 俯仰
    float roll;  // 翻滚
    float yaw;   // 偏航
} jy61p;
void read_offset(jy61p *jy61p_offset);
void read_angle(jy61p *jy61p_get, jy61p *jy61p_offset);
#endif
