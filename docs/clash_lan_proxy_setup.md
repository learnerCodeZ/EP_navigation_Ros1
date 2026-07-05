# 局域网共享 Clash VPN 给 Ubuntu

## 场景

Windows 电脑运行 Clash for Windows，让同一局域网下的 Ubuntu 电脑通过 Windows 的代理上网。

## 配置步骤

### 1. Clash 开启局域网访问

编辑 `C:\Users\<用户名>\.config\clash\cfw-settings.yaml`，添加：

```yaml
allow-lan: true
```

重启 CFW 后确认监听地址为 `0.0.0.0:7890`：

```powershell
netstat -ano | findstr "LISTENING" | findstr "7890"
```

### 2. Windows 防火墙放行

管理员 PowerShell 执行：

```powershell
netsh advfirewall firewall add rule name="Clash LAN" dir=in action=allow protocol=TCP localport=7890
```

### 3. 将网络设置为专用

这是**关键步骤**。如果网络被识别为公用（Public），防火墙默认会阻止外部连入。

管理员 PowerShell 执行：

```powershell
Set-NetConnectionProfile -Name "<WiFi名称>" -NetworkCategory Private
```

查看当前网络名称：

```powershell
Get-NetConnectionProfile | Select-Object Name, NetworkCategory
```

### 4. Ubuntu 配置代理

临时生效：

```bash
export http_proxy=http://192.168.123.191:7890
export https_proxy=http://192.168.123.191:7890
export all_proxy=socks5://192.168.123.191:7890
```

永久生效：

```bash
echo 'export http_proxy=http://192.168.123.191:7890' >> ~/.bashrc
echo 'export https_proxy=http://192.168.123.191:7890' >> ~/.bashrc
echo 'export all_proxy=socks5://192.168.123.191:7890' >> ~/.bashrc
source ~/.bashrc
```

### 5. 验证

```bash
curl -I https://www.google.com
```

返回 `200` 即成功。

### 6. Ubuntu 配置 Git 代理

Git 不会自动走系统代理，需要单独配置。

长期生效：

```bash
git config --global http.proxy http://192.168.123.191:7890
git config --global https.proxy http://192.168.123.191:7890
```

仅当前仓库生效（去掉 `--global`）：

```bash
git config http.proxy http://192.168.123.191:7890
git config https.proxy http://192.168.123.191:7890
```

取消代理：

```bash
git config --global --unset http.proxy
git config --global --unset https.proxy
```

## 常见问题

### Ubuntu 连不上代理（端口超时）

排查步骤：

1. `ping <Windows IP>` — 确认网络通
2. `nc -zv <Windows IP> 7890 -w 5` — 确认端口可达
3. 如果 ping 通但端口超时，大概率是 Windows 网络被识别为 Public

**最常见原因**：Windows 将 WiFi 识别为公用网络，公用配置文件的防火墙默认阻止外部连入。

**解决**：将网络改为 Private（见步骤 3）。

### 重连 WiFi 后又连不上

Windows 可能重新将网络识别为 Public。重新执行：

```powershell
Set-NetConnectionProfile -Name "<WiFi名称>" -NetworkCategory Private
```

### 防火墙规则丢失

重新添加：

```powershell
netsh advfirewall firewall add rule name="Clash LAN" dir=in action=allow protocol=TCP localport=7890
```

删除规则：

```powershell
netsh advfirewall firewall delete rule name="Clash LAN"
```

### Git HTTPS 通过代理 TLS 握手失败（高发问题）

> **记录日期**：2026-07-04

**现象**：

- `curl -I http://github.com` → 成功
- `curl https://github.com` → 失败（SSL 连接被重置）
- `git pull` → 失败（`gnutls_handshake` 错误）
- CONNECT 隧道能建立（返回 200），但之后 TLS 握手失败

**为什么是高发问题**：

- **Ubuntu 20.04 及更早版本默认用 GnuTLS 编译 git**，这是发行版级别的决策，所有用户都受影响，不是个例
- **代理场景下高发**，国内开发者用代理访问 GitHub 是刚需，遇到这个问题的概率很高
- 社区有大量讨论，搜索 "git gnutls_handshake proxy" 能找到无数同类问题，解决方案成熟

**也有一定特殊性**：

- 只在 Ubuntu 20.04 及更早版本出现，Ubuntu 22.04+ 已经改用 OpenSSL
- 只在通过 HTTP 代理访问 HTTPS 时触发，直连或 SOCKS 代理可能没事
- 容易被误诊为"Clash 不稳定"或"网络问题"，实际是 SSL 库兼容性问题

**关键判断**：如果 `curl`（用 OpenSSL）能通但 `git`（用 GnuTLS）不通，**不要怀疑网络或代理**，要怀疑底层 SSL 库差异。

**解决**：安装 PPA 版 git（默认用 OpenSSL 编译）：

```bash
sudo add-apt-repository ppa:git-core/ppa -y
sudo apt -o Acquire::http::Proxy="http://<Windows IP>:7890" update
sudo apt -o Acquire::http::Proxy="http://<Windows IP>:7890" install git -y
git --version  # 应该是 2.4x.x
```

详细原理记录见 `notes/git_https_proxy_issue.md`。
