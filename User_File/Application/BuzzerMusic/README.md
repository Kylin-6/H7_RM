# 蜂鸣器旋律

启用 `CHASSIS` 的板型在初始化完成且指示器可用时自动播放一次 GALA《Young For You》。
曲谱转录自用户提供的 `Buzzer-YOU/BSP/buzzer.c` 中 `gala_you()`，原文件署名 NCUROBOT (C) 2022。
音高取自其 `BSP/buzzer.h`，时长按原 `Note()` 的 `Long * 200 ms` 转换，不添加音间停顿。
播放开头 10 秒，响度 100%，达到 10 秒后截断静音。

现有 1 ms 任务调用 `BuzzerMusic_Update()`，按 HAL 毫秒时间推进，
不阻塞控制任务、不创建额外线程。直到重新上电不再播放。
使用本板 TIM12 蜂鸣器驱动，响度 100% 对应 50% PWM 占空比；实际音量取决于蜂鸣器及音高。
