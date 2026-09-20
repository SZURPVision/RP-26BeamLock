# SerialTest

串口协议联调工具，包含两个可执行文件：

| 目标 | 作用 |
| --- | --- |
| `serial_test` | 上位机侧：交互式发送目标角度/控制模式，实时打印电控回传的角度与编码器值 |
| `serial_simulate` | 电控侧：模拟云台电控，按协议周期回发角度/编码器值，并解析上位机下发的指令 |

协议格式见 `docs/PROTOCOL.md`。没有真实电控时，用 `socat` 造一对虚拟串口即可完整联调：

```bash
# 终端 1：建立一对虚拟串口，/tmp/ttyV0 给主程序，/tmp/ttyV1 给模拟器
socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1

# 终端 2：启动电控模拟器
./build/Tools/SerialTest/serial_simulate --port /tmp/ttyV1

# 终端 3：把 config.yaml 里的 SerialConfig.port_name 改为 /tmp/ttyV0 后运行主程序，
#         或单独运行 serial_test 手动发指令
./build/Tools/SerialTest/serial_test
```

## 构建

```bash
cmake -S . -B build
cmake --build build --target serial_test serial_simulate -j
```

## serial_test 指令

```
send <pitch> <yaw> [gyro|encoder|reset]   设置目标角度与控制模式
control <on|off>                          电机是否发力
reset                                     发送回正指令
status                                    打印当前角度与编码器值
toggle                                    开关每秒自动打印状态
wait                                      等待收到第一帧电控反馈
q                                         退出
```

## serial_simulate 指令

```
p <value> / y <value>    设置模拟的当前 pitch / yaw
t <pitch> <yaw>          设置目标角度（模拟电控内部的闭环目标）
rate <value>             设置跟随速度（deg/s）
follow on|off            是否跟随目标
status                   打印模拟器状态
q                        退出
```

参数：`--port <设备>`（默认 `/dev/ttyUSB0`）、`--baud <波特率>`（默认 115200）、`--hz <回发频率>`（默认 50）。
