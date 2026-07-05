
# RoboMaster EP & Jetson Xavier 远程控制手册

## 1. 网络连接
打开手机热点，确保 **Xavier** 和 **笔记本 (Windows)** 都连接到同一个热点。

## 2. 查看 IP 地址

### 查看 Xavier 网络
```bash
hostname -I
```
示例输出：
```
192.168.42.3 10.172.20.241 240e:441:9a24:30d8:6846:f1f:2aec:7871 240e:441:9a24:30d8:1ae5:8d62:f0a6:f6de
```
- `192.168.42.3` 通常是通过 USB 线直连小车的 RNDIS 网络（**不能用于和 Windows 通信**）
- `10.172.20.241` 是连接手机热点获得的局域网 IP（**用于和 Windows 通信**）
- 后面带冒号的是 IPv6 地址，忽略

**⚠️ 注意**：具体哪个 IP 是热点 IP，需要对比 Windows 的 IP 网段来判断。

### 查看 Windows 网络
在 Windows 的 CMD 或 PowerShell 中执行：
```cmd
ipconfig
```
找到 **无线局域网适配器 WLAN** 下的 IPv4 地址，例如：
```
IPv4 地址 . . . . . . . . . . . . : 10.172.20.24
```
如果 Xavier 的热点 IP 也是 `10.172.20.x` 网段，说明两者在同一网络，可以相互通信。

## 3. 安装必要软件

### 在 Windows 上
- **VS Code** + **Remote-SSH 插件**：用于远程编辑 Xavier 上的代码
- **VcXsrv**：用于显示 Xavier 上运行的图形界面（如 RViz）
  - 下载：https://sourceforge.net/projects/vcxsrv/
  - 启动 XLaunch，配置：
    - Display number：手动改为 **0**（默认是 -1，必须改）
    - 勾选 **Disable access control**（否则远程连接被拒）
    - 其他默认，完成。系统托盘会显示 X 图标。

### 在 Xavier 上
```bash
sudo apt update
sudo apt install tmux   # 可选，终端复用器用于持久化会话
```

## 4. 配置 ROS 环境变量

在 Xavier 的 `~/.bashrc` 文件中添加（**用实际的热点 IP 替换**）：
```bash
export ROS_MASTER_URI=http://10.172.20.241:11311
export ROS_IP=10.172.20.241
```
然后执行：
```bash
source ~/.bashrc
```
**验证**：
```bash
echo $ROS_MASTER_URI   # 应输出 http://10.172.20.241:11311
echo $ROS_IP           # 应输出 10.172.20.241
```
>- export ROS_MASTER_URI=http://10.172.20.241:11311
作用：设置 ROS Master 的地址。
>- export ROS_IP=10.172.20.241
作用：告诉其他节点 “用这个 IP 来连接我”。
>-  因为 Xavier 同时作为 ROS Master 和数据发布者，这两个变量都填 Xavier 自己的热点 IP。

## 5. X11 图形转发 (让 RViz 显示在 Windows 上)

在启动任何图形程序的终端中，**每次**都要先设置 `DISPLAY` 变量：
```bash
export DISPLAY=10.172.20.24:0.0
```
- `10.172.20.24` 是 Windows 的 IP
- `:0.0` 是 VcXsrv 的显示编号（与 XLaunch 中设置的 Display number 一致）

**验证 X11 是否通**：
```bash
xeyes
```
如果出现一双眼睛跟随鼠标，说明转发成功。

## 6. 远程开发（修改代码）

### 使用 VS Code SSH 远程
1. 在 VS Code 中安装 **Remote - SSH** 插件
2. 按 `F1` → 选择 `Remote-SSH: Connect to Host...` → 输入 `用户名@10.172.20.241`
3. 打开后即可像本地一样编辑 Xavier 上的文件，内置终端也是 Xavier 的环境

**注意**：VS Code 远程终端可能不会自动 source `~/.bashrc`，建议新开终端后先手动执行：
```bash
source ~/.bashrc
```



## 7. 使用 tmux 保护后台进程
网络断开或关闭终端后，roscore / 键盘控制等进程会继续运行。
```bash
tmux new -s roscore
source ~/.bashrc
roscore
```
按 `Ctrl+B` 然后按 `D` 断开会话。  
重新连接：
```bash
tmux attach -t roscore
```

## 8. 常见问题

### RViz 窗口不弹出 / 报 `could not connect to display`
- 检查 Windows 上 VcXsrv 是否运行，Display number 是否设为 0，是否勾选“Disable access control”
- 检查 Windows 防火墙是否拦截（可临时关闭防火墙测试）
- 确认 `export DISPLAY=Windows_IP:0.0` 已执行且 IP 正确

### RViz 弹出了但看不到地图/激光数据
- 确保 Xavier 上 `ROS_MASTER_URI` 和 `ROS_IP` 是热点 IP（不是 `localhost` 或 `127.0.0.1`）
- 检查 `rostopic list` 查看是否有话题数据
- 在 RViz 中添加正确的显示类型（如 LaserScan、Map），并选择对应的话题和 Fixed Frame

### 键盘控制提示 “Waiting for subscriber to connect to /cmd_vel”
- 需要先启动小车底盘驱动节点（通常在 mapping.launch 或单独的 rm_ep_driver.launch 中）
- 如果单独键盘测试，可以用 `rostopic echo /cmd_vel` 模拟一个订阅者来激活键盘节点

### 小车连接失败 (`Connection refused` 或 camera timeout)
- 检查 Xavier 与小车之间的 USB 线是否插好
- 在 Xavier 上 `ping 192.168.42.2` 确认 RNDIS 网络通不通
- 暂时禁用相机启动：在 launch 文件中设置 `enable_camera` 为 `false`
- 若频繁出现，尝试重启 EP 小车

### 导航时循环提示 “Request for map failed”
- 确认已执行 `map_saver` 保存地图，并且导航 launch 文件的 `map_server` 参数指向了正确的 `.yaml` 文件
- 确认 `roscore` 运行正常，且 `map_server` 节点启动成功

---

> 最后更新：2026-06-01  
> 适用环境：Jetson Xavier NX (Ubuntu 20.04) + Windows 11 + RoboMaster EP
