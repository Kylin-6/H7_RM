#ifndef KEYBOARD_LINK_H
#define KEYBOARD_LINK_H

#include "keyboard_protocol.h"

bool KeyboardLink_Init(bool receive);
bool KeyboardLink_Send(Struct_Keyboard_Frame frame);
/** 返回是否曾收到新合法帧；过期快照仍返回，调用方必须核对 received_ms。 */
bool KeyboardLink_Read(Struct_Keyboard_Frame& frame);

#endif
