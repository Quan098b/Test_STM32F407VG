#include "Thu_vien_Mecanum.h"

static int16_t iabs16(int16_t x) { return (x < 0) ? (int16_t)(-x) : x; }

void Mecanum_ModeCoeffs(Mecanum_Mode_t mode, int8_t coeff[4])
{
    /* coeff order: FL, FR, RL, RR */
    int8_t fl = 0, fr = 0, rl = 0, rr = 0;

    switch (mode)
    {
        case MEC_MODE_FWD:
            fl = +1; fr = +1; rl = +1; rr = +1;
            break;

        case MEC_MODE_BACK:
            fl = -1; fr = -1; rl = -1; rr = -1;
            break;

        case MEC_MODE_LEFT:
            /* vx<0, vy=0 => FL=-, FR=+, RL=+, RR=- */
            fl = -1; fr = +1; rl = +1; rr = -1;
            break;

        case MEC_MODE_RIGHT:
            /* vx>0, vy=0 => FL=+, FR=-, RL=-, RR=+ */
            fl = +1; fr = -1; rl = -1; rr = +1;
            break;

        case MEC_MODE_DIAG_FL:
            /* vy=+1, vx=-1 => FL=0, FR=+, RL=+, RR=0 */
            fl = 0;  fr = +1; rl = +1; rr = 0;
            break;

        case MEC_MODE_DIAG_FR:
            /* vy=+1, vx=+1 => FL=+, FR=0, RL=0, RR=+ */
            fl = +1; fr = 0;  rl = 0;  rr = +1;
            break;

        case MEC_MODE_DIAG_BL:
            /* vy=-1, vx=-1 => FL=-, FR=0, RL=0, RR=- */
            fl = -1; fr = 0;  rl = 0;  rr = -1;
            break;

        case MEC_MODE_DIAG_BR:
            /* vy=-1, vx=+1 => FL=0, FR=-, RL=-, RR=0 */
            fl = 0;  fr = -1; rl = -1; rr = 0;
            break;

        default:
            fl = 0; fr = 0; rl = 0; rr = 0;
            break;
    }

    coeff[0] = fl;
    coeff[1] = fr;
    coeff[2] = rl;
    coeff[3] = rr;
}

void Mecanum_Mix(int16_t vx, int16_t vy, int16_t out[4])
{
    /* out: FL, FR, RL, RR */
    int16_t fl = (int16_t)(vy + vx);
    int16_t fr = (int16_t)(vy - vx);
    int16_t rl = (int16_t)(vy - vx);
    int16_t rr = (int16_t)(vy + vx);

    int16_t m1 = iabs16(fl);
    int16_t m2 = iabs16(fr);
    int16_t m3 = iabs16(rl);
    int16_t m4 = iabs16(rr);

    int16_t maxv = m1;
    if (m2 > maxv) maxv = m2;
    if (m3 > maxv) maxv = m3;
    if (m4 > maxv) maxv = m4;

    if (maxv > 255)
    {
        /* chuẩn hoá về biên 255 */
        fl = (int16_t)((fl * 255) / maxv);
        fr = (int16_t)((fr * 255) / maxv);
        rl = (int16_t)((rl * 255) / maxv);
        rr = (int16_t)((rr * 255) / maxv);
    }

    out[0] = fl;
    out[1] = fr;
    out[2] = rl;
    out[3] = rr;
}
