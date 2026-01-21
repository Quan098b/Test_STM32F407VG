#ifndef THU_VIEN_MECANUM_H_
#define THU_VIEN_MECANUM_H_

#include <stdint.h>

/*
  Quy ước trục robot (chuẩn trong mecanum):
  - +Vy: tiến (forward)
  - +Vx: sang phải (strafe right)
  - Wz: quay (không dùng trong yêu cầu này, nhưng có thể mở rộng)

  Thứ tự bánh trong thư viện:
    [0] FL: Front-Left  (trái trước)
    [1] FR: Front-Right (phải trước)
    [2] RL: Rear-Left   (trái sau)
    [3] RR: Rear-Right  (phải sau)

  Mode rời rạc được ánh xạ theo mixer:
    wFL = Vy + Vx
    wFR = Vy - Vx
    wRL = Vy - Vx
    wRR = Vy + Vx
  (Wz=0 trong các mode dưới)
*/

typedef enum
{
    MEC_MODE_FWD = 0,      /* tiến */
    MEC_MODE_BACK,         /* lùi */

    MEC_MODE_LEFT,         /* trượt trái */
    MEC_MODE_RIGHT,        /* trượt phải */

    MEC_MODE_DIAG_FL,      /* chéo: tiến-trái */
    MEC_MODE_DIAG_FR,      /* chéo: tiến-phải */
    MEC_MODE_DIAG_BL,      /* chéo: lùi-trái */
    MEC_MODE_DIAG_BR       /* chéo: lùi-phải */
} Mecanum_Mode_t;

/*
  Hệ số hướng quay theo từng mode:
  coeff[i] ∈ {-1,0,+1}
  -1: bánh quay nghịch (REV)
  +1: bánh quay thuận (FWD)
   0: bánh dừng (coast/disable)
*/
void Mecanum_ModeCoeffs(Mecanum_Mode_t mode, int8_t coeff[4]);

/*
  Mixer tổng quát (tùy chọn):
  vx, vy ∈ [-255..255] (tỷ lệ hoá)
  out[i] ∈ [-255..255] sau chuẩn hoá biên độ (max-abs = 255)
*/
void Mecanum_Mix(int16_t vx, int16_t vy, int16_t out[4]);

#endif /* THU_VIEN_MECANUM_H_ */
