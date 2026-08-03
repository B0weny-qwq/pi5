# SSH手动电机命令

`motor_cli`直接使用ZDT `0xFD`梯形相对位置模式。角度是电机轴角度，单位为度，
不是水管角度；默认最大速度为6 RPM，加速和减速均为200 RPM/s。

运行手动命令前先退出`ball2_task3_velocity`。两个程序使用同一个UART互斥锁，主控
仍在运行时，手动命令会报`ZDT UART is busy`，不会发送串口命令。

## 登录后使用

```bash
cd /home/boweny/111.3_p
./motor_cli status
./motor_cli up 5
./motor_cli down 5
./motor_cli move 5
./motor_cli move -5
./motor_cli stop
./motor_cli zero
./motor_cli enable
./motor_cli disable
```

- `up 5`：按当前`motorSign`定义的升高方向，相对转动5度。
- `down 5`：按当前`motorSign`定义的降低方向，相对转动5度。
- `move +/-5`：直接使用电机正负方向，不经过`motorSign`语义转换。
- `status`：只读位置、速度、使能、到位、堵转和编码器状态，不使能电机。
- `zero`：先停止，再把当前位置设为逻辑零点；不会主动转动电机。
- `enable`：只使能并保持当前位置，不清零、不主动转动。
- `disable`：停止后释放保持力矩，机械机构可能在重力下移动。

手动相对运动限制在1到10度。运动过程中检测到堵转、保护、超时或Ctrl+C时，工具会
发送立即停止命令。

## Windows PowerShell一条命令执行

```powershell
ssh boweny@10.97.228.38 '/home/boweny/111.3_p/motor_cli status'
ssh boweny@10.97.228.38 '/home/boweny/111.3_p/motor_cli up 5'
ssh boweny@10.97.228.38 '/home/boweny/111.3_p/motor_cli down 5'
ssh boweny@10.97.228.38 '/home/boweny/111.3_p/motor_cli stop'
```
