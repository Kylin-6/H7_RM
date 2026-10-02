#ifndef DM_IMU_INS_H
#define DM_IMU_INS_H

#include <stdbool.h>
#ifdef __cplusplus
extern "C"
{
#endif
    /** 初始化唯一 DM-IMU 设备；BMI088 与本桥由板型互斥选择。 */
    bool DM_IMU_InsBridge_Init(void);
    /** 1 kHz 请求欧拉角；仅新鲜的新帧发布 INS，Pitch 差分速度放 gyro_y_rad_s。 */
    void DM_IMU_InsBridge_Update(void);
    /** 最近 100 ms 是否收到合法欧拉角；其他寄存器帧不能续期。 */
    bool DM_IMU_InsBridge_IsFresh(void);
#ifdef __cplusplus
}
#endif
#endif
