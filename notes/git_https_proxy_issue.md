# 局域网共享 Clash VPN + Ubuntu 代理问题排查全记录

> **记录日期**：2026-07-04

## 背景

我的开发环境：Windows 电脑运行 Clash for Windows（VPN），另一台 Ubuntu 20.04（Jetson 设备）在同一局域网下，想让 Ubuntu 也能通过 Windows 的 Clash 上网和访问 GitHub。

经过了一整天的排查，遇到了多个问题，以下是完整记录。

---

## 问题一：Clash 不监听局域网

### 现象

Ubuntu 上 `curl` 走代理超时，Windows 上 `netstat` 显示 Clash 只监听 `127.0.0.1:7890`，没有监听 `0.0.0.0:7890`。

### 原因

直接编辑 `config.yaml` 加 `allow-lan: true` 会被 Clash for Windows 的 profiles 机制覆盖，CFW 有自己的配置管理，启动时会重新生成 `config.yaml`。

### 解决

在 `C:\Users\<用户名>\.config\clash\cfw-settings.yaml` 中添加：

```yaml
allow-lan: true
```

重启 CFW 后确认监听地址变为 `0.0.0.0:7890`。

### 学到的

- CFW 的 `config.yaml` 是自动生成的，手动改会被覆盖
- `cfw-settings.yaml` 才是 CFW 的持久化配置
- `127.0.0.1` 只接受本机连接，`0.0.0.0` 才接受局域网连接

---

## 问题二：Windows 防火墙阻止外部连入

### 现象

Ubuntu 能 ping 通 Windows，但 `nc -zv 192.168.123.191 7890` 超时。

### 原因

Windows Defender 防火墙默认阻止外部设备连入。虽然添加了端口 7890 的入站规则，但仍然不通。

进一步排查发现：**网络被 Windows 识别为"公用网络"（Public）**，公用配置文件的防火墙规则更严格，默认阻止所有外部连入。

### 解决

1. 添加防火墙规则（管理员 PowerShell）：

```powershell
netsh advfirewall firewall add rule name="Clash LAN" dir=in action=allow protocol=TCP localport=7890
```

2. 将网络改为专用（**关键步骤**）：

```powershell
Set-NetConnectionProfile -Name "<WiFi名称>" -NetworkCategory Private
```

查看当前网络：

```powershell
Get-NetConnectionProfile | Select-Object Name, NetworkCategory
```

### 学到的

- Windows 网络分三种配置文件：域、专用、公用，各有独立防火墙规则
- **公用网络**默认更严格，即使你加了允许规则，默认阻止规则可能优先级更高
- 重连 WiFi 后 Windows 可能重新识别为 Public，需要重新设置
- 按程序放行（`program=...`）不一定比按端口放行更可靠，这次两种都没生效，根本原因是网络类型

---

## 问题三：Git HTTPS 通过代理 TLS 握手失败

### 现象

- `curl -I http://github.com` → 成功（HTTP 200）
- `curl https://github.com` → 失败（SSL 连接被重置）
- `git pull` → 失败（gnutls_handshake 错误）
- CONNECT 隧道能建立（200 Connection established），但之后 TLS 握手失败

### 原因

Ubuntu 20.04 自带的 git 2.25.1 使用 **GnuTLS** 作为 SSL/TLS 库，与 Clash 代理的 HTTPS 转发存在兼容性问题。

### OpenSSL vs GnuTLS

两者都是 SSL/TLS 库，负责加密通信（如 HTTPS）：

| | OpenSSL | GnuTLS |
|---|---|---|
| 来源 | OpenSSL 项目 | GNU 项目 |
| 使用范围 | 最主流，大多数软件默认使用 | 部分 Linux 发行版默认使用 |
| 兼容性 | 好 | 差，某些代理场景下容易出问题 |
| 开源协议 | Apache 2.0 | LGPL（更"纯粹"的开源） |

简单理解：都是干同一件事的工具，但 OpenSSL 更稳，GnuTLS 容易出兼容问题。Ubuntu 20.04 出于开源协议考虑默认用 GnuTLS 编译 git，但在代理场景下容易翻车。

### 解决

安装 PPA 版 git，默认使用 OpenSSL 编译：

```bash
sudo add-apt-repository ppa:git-core/ppa -y
sudo apt -o Acquire::http::Proxy="http://192.168.123.191:7890" update
sudo apt -o Acquire::http::Proxy="http://192.168.123.191:7890" install git -y
```

验证：

```bash
git --version
# 2.50.1 → 升级成功，使用 OpenSSL
```

### 学到的

- Ubuntu 20.04 自带的 git 用 GnuTLS，这就是为什么 curl（用 OpenSSL）能通但 git 不行
- 同样是 HTTPS 请求，底层用的 SSL 库不同，结果可能不一样
- `apt -o Acquire::http::Proxy="..."` 可以让 apt 也走代理，PPA 服务器在国外，不走会很慢
- 排查 HTTPS 问题时，要关注底层用的是哪个 TLS 库，不能只看"HTTPS 不通"

---

## 完整配置速查

### Windows 端（一次性配置）

```powershell
# 1. cfw-settings.yaml 加 allow-lan: true，重启 CFW

# 2. 防火墙放行
netsh advfirewall firewall add rule name="Clash LAN" dir=in action=allow protocol=TCP localport=7890

# 3. 网络改专用
Set-NetConnectionProfile -Name "PDCN_5G" -NetworkCategory Private
```

### Ubuntu 端（一次性配置）

```bash
# 1. 系统代理
echo 'export http_proxy=http://192.168.123.191:7890' >> ~/.bashrc
echo 'export https_proxy=http://192.168.123.191:7890' >> ~/.bashrc
echo 'export all_proxy=socks5://192.168.123.191:7890' >> ~/.bashrc
source ~/.bashrc

# 2. Git 代理
git config --global http.proxy http://192.168.123.191:7890
git config --global https.proxy http://192.168.123.191:7890

# 3. 升级 git（解决 GnuTLS 兼容问题）
sudo add-apt-repository ppa:git-core/ppa -y
sudo apt -o Acquire::http::Proxy="http://192.168.123.191:7890" update
sudo apt -o Acquire::http::Proxy="http://192.168.123.191:7890" install git -y
```

### 验证

```bash
curl -I https://www.google.com    # 测试代理是否通
git ls-remote https://github.com/learnerCodeZ/EP_navigation_Ros1.git  # 测试 git 是否通
```
