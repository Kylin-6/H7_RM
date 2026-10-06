#!/usr/bin/env python3
"""不暂停 MCU，通过 StorageTask 导出 W25Q64 日志；校验并转换为 CSV。"""
import argparse
import csv
import socket
import struct
import subprocess
import tempfile
import time
import zlib
from pathlib import Path

MAGIC = 0x31474C43
RECORD = struct.Struct('<QII22f4I')
FIELDS = ['timestamp_us', 'sequence', 'flags', 'yaw_target_rad', 'yaw_actual_rad',
          'yaw_error_rad', 'yaw_stick_rad_s', 'yaw_pid_rad_s', 'base_rad_s',
          'yaw_ff_rad_s', 'yaw_command_rad_s', 'yaw_motor_rad_s', 'yaw_kd',
          'yaw_torque_nm', 'vx_m_s', 'vy_m_s', 'w_rad_s', 'planned_x', 'planned_y',
          'planned_w', 'wheel0_rad_s', 'wheel1_rad_s', 'wheel2_rad_s', 'wheel3_rad_s',
          'yaw_position_rad', 'motor_states', 'gimbal_age_us', 'chassis_age_us', 'fault_mask']


def decode(source, destination):
    data = Path(source).read_bytes()
    if len(data) % 256:
        raise ValueError('日志长度不是 256 byte 页的整数倍')
    records = 0
    bad_pages = 0
    session = -1
    with Path(destination).open('w', newline='') as output:
        writer = csv.writer(output)
        writer.writerow(['session', 'page'] + FIELDS)
        for offset in range(0, len(data), 256):
            page = data[offset:offset + 256]
            if page == b'\xff' * 256:
                continue
            magic, count, size, index, crc = struct.unpack_from('<IHHII', page)
            if (magic != MAGIC or size != RECORD.size or count not in (1, 2) or
                    index != offset // 256 or zlib.crc32(page[16:]) != crc):
                bad_pages += 1  # 掉电未写完的页不得当成有效样本。
                continue
            for slot in range(count):
                row = RECORD.unpack_from(page, 16 + slot * RECORD.size)
                if row[1] == 0 or session < 0:
                    session += 1
                writer.writerow([session, index] + list(row))
                records += 1
    print(f'有效样本 {records}，损坏/不兼容页 {bad_pages}，CSV：{destination}')


class OpenOCD:
    def __init__(self, port):
        self.socket = socket.create_connection(('127.0.0.1', port), 5)
        self.socket.settimeout(10)

    def command(self, text):
        self.socket.sendall(text.encode() + b'\x1a')
        response = b''
        while b'\x1a' not in response:
            chunk = self.socket.recv(65536)
            if not chunk:
                raise ConnectionError('OpenOCD 断开连接')
            response += chunk
        return response.split(b'\x1a')[0].decode()

    def read(self, address, length):
        data = bytearray()
        # DAP 大块字节读取可能失败；分成小块，优先使用对齐的 32 bit 访问。
        for offset in range(0, length, 128):
            count = min(128, length - offset)
            aligned = (address + offset) % 4 == 0 and count % 4 == 0
            width = 32 if aligned else 8
            words = count // 4 if aligned else count
            response = self.command(f'read_memory {address + offset:#x} {width} {words}')
            values = response.split()
            if len(values) != words:
                raise RuntimeError(f'读取不完整，地址 {address + offset:#x}：{response[:200]}')
            for value in values:
                data.extend(int(value, 0).to_bytes(width // 8, 'little'))
        return bytes(data)

    def write(self, address, value):
        response = self.command(f'write_memory {address:#x} 32 {{{value}}}')
        if response.strip():
            raise RuntimeError(f'写入调试邮箱失败：{response}')

    def wait(self, debug):
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            state = struct.unpack('<8I', self.read(debug, 32))
            if state[4] == 0:
                return state
            time.sleep(.02)
        raise TimeoutError('StorageTask 未响应；请检查任务/Flash 状态')


def dump(elf, port, destination):
    symbols = {}
    for line in subprocess.check_output(['arm-none-eabi-nm', '-n', str(elf)], text=True).splitlines():
        parts = line.split()
        if len(parts) == 3:
            symbols[parts[2]] = int(parts[0], 16)
    debug = symbols['ChassisFlashLog_Debug']
    buffer = symbols['ChassisFlashLog_ReadBuffer']
    target = OpenOCD(port)
    try:
        # 先核对固件代码，避免将邮箱写进旧版本的其他变量。
        with tempfile.TemporaryDirectory() as directory:
            text = Path(directory) / 'text.bin'
            subprocess.run(['arm-none-eabi-objcopy', '--dump-section', f'.text={text}', str(elf), str(Path(directory) / 'elf-copy')], check=True)
            sections = subprocess.check_output(['arm-none-eabi-objdump', '-h', str(elf)], text=True)
            text_address = next(int(line.split()[3], 16) for line in sections.splitlines()
                                if len(line.split()) > 4 and line.split()[1] == '.text')
            code_address = symbols['_Z19ChassisFlashLog_Runv'] & ~1
            code_offset = code_address - text_address
            expected = text.read_bytes()[code_offset:code_offset + 256]
            if len(expected) != 256 or target.read(code_address, len(expected)) != expected:
                raise RuntimeError('ELF 与板上固件不匹配，未写入邮箱')
        state = struct.unpack('<8I', target.read(debug, 32))
        if state[0] == 0 or state[4] != 0:
            raise RuntimeError('记录器正在初始化或处理其他请求，请稍后重试')
        target.write(debug + 16, 1)  # 停止采样并排空 RAM，电机控制继续运行。
        state = target.wait(debug)
        if state[0] in (0, 1, 2):
            raise RuntimeError('记录器未确认停止，未开始导出')
        length = state[1]
        with Path(destination).open('wb') as output:
            for address in range(0, length, 4096):
                size = min(4096, length - address)
                target.write(debug + 20, address)
                target.write(debug + 24, size)
                target.write(debug + 28, 0xffffffff)
                target.write(debug + 16, 2)
                response = target.wait(debug)
                if response[7] != 0 or response[5] != address or response[6] != size:
                    raise RuntimeError(f'Flash 读取失败，地址 {address:#x}，结果 {response[7]}')
                output.write(target.read(buffer, size))
        print(f'已导出 {length} byte：{destination}；记录已停止，控制仍运行')
    finally:
        target.socket.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    export = commands.add_parser('dump')
    export.add_argument('--elf', type=Path, required=True)
    export.add_argument('--port', type=int, default=6666)
    export.add_argument('--output', type=Path, required=True)
    convert = commands.add_parser('decode')
    convert.add_argument('source', type=Path)
    convert.add_argument('destination', type=Path)
    args = parser.parse_args()
    if args.command == 'dump':
        dump(args.elf, args.port, args.output)
    else:
        decode(args.source, args.destination)


if __name__ == '__main__':
    main()
