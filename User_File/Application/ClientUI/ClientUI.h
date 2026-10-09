#ifndef CLIENT_UI_H
#define CLIENT_UI_H

bool ClientUI_Init();
// ControlTask 在各机构更新后调用；只采集一致显示快照，不做 I/O。
void ClientUI_Capture();
// StatusTask 100 Hz 调用，内部降频且非阻塞。
void ClientUI_Update();

#endif
