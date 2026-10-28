# 网页调试界面（「设置」面板）修复说明

> 对应文件：`mpy_modules/camera_on.py`（上电自动执行的开关文件，含服务端修复）
> 本地无机器调试工具：`tools/web_ui_mock.py`
> 状态：**已在硬件上完整回归**（2026-10-28）

---

## 一、现象

网页遥控页（`http://192.168.4.1/`）顶部「设置」菜单里的项目点了没反应，
进不去对应的功能面板：

| 菜单项 | 面板 |
|---|---|
| 🎯 舵机校准 | 8 路舵机零偏校准台（滑块 / ±微调 / 写入 NVS） |
| 🧭 IMU 校准 | 加速度计 + 陀螺仪零偏标定 |
| ⚙️ 运动参数 | 速度 / 步幅 / 站立高度 |
| 💾 持久化参数 | NVS 里的 8 路零偏（只读） |
| ℹ️ 系统信息 | 芯片 / 存储 / 电池 / 内存 / 固件版本 |
| 🌐 语言设置 | 界面语言 |

用户侧表现为**时好时坏**：第一次能进、第二次进不去；有时连「设置」下拉都打不开；
断电重启后又能用——但无法保证每次点击都生效。

---

## 二、根因（三个，按发现顺序）

### 根因 1：改错了地方 —— 补丁没进真正发出的页面

`frozen/camera_stream.py` 的 HTTP 处理器是用 **`_html_page()`** 生成响应体的：

```python
else:
    _send_html(client)        # → _html_page() → 真正发给浏览器的 HTML
```

只改 `camera_stream._HTML_TEMPLATE` **不会**改变实际响应（该属性在运行期已被
`_html_page()` 读取/拼装，模块属性替换对它无效）。所以早期"注入了 5 个模态框、
打印 ✓ 成功"的部署，页面上其实什么都没有。

**修复**：包装 `camera_stream._html_page()`，把 UI 片段注入**真实响应体**。

### 根因 2：注入的 JS 有语法错误，整个脚本块报废

旧注入代码在单引号字符串里嵌套了未转义的单引号：

```js
h += '... oninput="document.getElementById('v' + i + '').innerText=..." ...';
//                                                        ^ 字符串在这里就断了
```

`node --check` 直接报 `SyntaxError: Unexpected identifier 'v'`。
**一个 script 块语法错误，整个块都不执行** —— `openModal()` 从未被定义，
点菜单自然全线无响应。

**修复**：不再用行内 `onclick`/`ontouchend` 拼字符串；改为
`data-bp` / `data-cmd` / `data-close` 属性 + **捕获阶段事件委托**，
从结构上消除引号嵌套问题。所有 JS 在部署前都用 `node --check` 验过。

### 根因 3（核心）：菜单按钮在文档顶部，绑定脚本在文档底部 + 服务端 2 秒阻塞

原页面结构：**菜单按钮在 `<body>` 很靠前的位置，而给它绑 `click` 的
`<script>` 在文档最末尾**。同时 `frozen/camera_stream.py` 的接收循环是
**单线程 + 阻塞读**：

```python
client.settimeout(2.0)
raw = client.recv(512)        # 只读一次，最多 512 字节
```

手机浏览器会开预连接、会取消请求，这些"连上但没发完请求"的连接会把
**整个服务器阻塞满 2 秒**（实测：正常 144ms → 被拖到 2119ms）。
页面于是慢慢吐出来，用户看到顶部菜单就点——**此刻底部绑定脚本还没执行，
按钮上没有任何处理函数，点击必然无响应**。等页面加载完再点就好了，
这正是"时好时坏"的来源。

**修复（`mpy_modules/camera_on.py`）**：

1. UI 注入位置从 `</body>` 前移到 **`<body>` 之后**，尽早执行；
2. **菜单按钮也改为捕获阶段事件委托**（绑在 `document` 上）——
   文档一解析出按钮就能用，与底部脚本是否执行完**完全解耦**；
3. 菜单项的动作在**点击时**按文本/位置解析，不再依赖加载时的预绑定；
4. 降低连接压力：看门狗轮询 3s→10s，页面自身速度同步 1.5s→5s
   （该服务器每个请求都要新建 TCP 连接，请求越密越容易把服务拖住）。

> **验证方法**：把页面自身底部 `<script>` 整个删掉（模拟"加载到一半就点"），
> 修复前菜单完全点不动；修复后开关菜单、开校准面板、8 张卡片、反复开关全部正常。

---

## 三、顺带修掉的问题

| 问题 | 原因 | 处理 |
|---|---|---|
| 手机一直显示旧页面 | 固件响应**没有任何缓存头** | 响应加 `Cache-Control: no-store, no-cache, must-revalidate` + `Pragma: no-cache` + `Expires: 0`，`<head>` 内同步加 meta |
| 舵机校准突然全部失效，报 `ERR:'socket' object has no attribute 'get_cal_point'` | 固件用 `exec(代码, globals())` 把 `/camera_on.py`、`/pwm_ext_on.py`、KittenBlock 的 `/main.py`、串口 REPL、`mpremote run` 脚本**全部塞进同一个命名空间**；任何一处出现 `s = ...` 就把舵机模块覆盖掉 | 补丁改为**闭包 + 函数内 import**，不依赖任何可被覆盖的全局名。实测把 `s`/`page`/`get`/`cal`/`voltage`/`gc`/`time` 等 17 个名字污染成垃圾值后接口依然正常 |
| IMU 校准点了没反应 | 旧代码调 `calib_mag.run()`（不存在）；且 IMU 是**懒初始化**，不 `init()` 时 `calibrate()` 会**静默什么都不做**却提示成功 | 改为 `bpuppy_imu.init(0,14,21,0x68)` → 检查 `is_ready()` → `calibrate(300)`（≈3 秒）；检测不到 IMU 时明确报错 |
| 指南小狗开关永远只关不开 | `heading_follow.running` 是**函数**不是布尔值，判断恒真 | 改为调用 `running()`；`start()` 需要目标航向参数，做成面板传 `deg` |
| 板端报错却提示"成功" | `api()` 一律显示传入的成功文案；且 `fetch` 在 HTTP 502 时**不 reject** | 响应以 `ERR:` 开头时显示真实错误；`fetch` 增加 `r.ok` 判断 |
| 板子掉线时界面静默卡住 | 无任何提示 | 加**连接看门狗**：底部红色横幅「⚠ 机器狗无响应」；各面板加载失败时明确说明 |
| `/cmd` 响应可能被浏览器缓存 | 无缓存头 | 请求加一次性参数 `&_bp=<时间戳>` |

---

## 四、部署

文件就是上电开关文件本身：**存在即执行，不存在则跳过**，下次上电生效。

```bash
# 1. 让板子挂载文件系统（frozen main.py 会跑一遍，顺手把 VFS 挂上）
mpremote connect /dev/cu.usbmodemXXXX resume exec "import main"

# 2. 覆盖（先用 connect 那条，再用 resume 保持挂载）
mpremote connect /dev/cu.usbmodemXXXX resume cp mpy_modules/camera_on.py :/camera_on.py

# 3. 校验（本地与板子 SHA-256 必须一致）
shasum -a 256 mpy_modules/camera_on.py
mpremote connect /dev/cu.usbmodemXXXX resume exec \
  "import hashlib,binascii;print(binascii.hexlify(hashlib.sha256(open('/camera_on.py','rb').read()).digest()).decode())"

# 4. 重启
mpremote connect /dev/cu.usbmodemXXXX resume exec "import machine; machine.reset()"
```

> ⚠️ **必须用 USB + mpremote 传**。文件里有中文界面文案，走蓝牙/ViperIDE 传输会丢
> 非 ASCII 字节导致文件损坏（这与 `pwm_ext_on.py` 必须纯 ASCII 是同一条限制）。
>
> ⚠️ 摄像头默认**不开机自启**（`camera_stream.start()` 不带 `stream=True`），
> 与出厂版 `camera_on.py` 行为一致与否见 `docs/wifi设备遥控指南.md` 2.3；
> 想恢复开机即图传，把文件末尾改成 `camera_stream.start(stream=True)`。

---

## 五、不连机器狗也能调界面

```bash
python3 tools/web_ui_mock.py                # 先从狗子拉一次真实页面，之后可离线
python3 tools/web_ui_mock.py --offline      # 只用本地缓存
```

本地起一个同页面 + 有状态 `/cmd` mock 的服务，浏览器开
`http://127.0.0.1:9000/` 就能点全部面板；校准值、运动参数改动都会真的变。
改完 `camera_on.py` 重新拉一次页面即可验证，**全程不碰机器狗**。

> 浏览器可能缓存旧页面（换了内容行为不对时，加 `/?v=2` 或换端口）。

---

## 六、诊断手段（页面自己会汇报）

`camera_on.py` 内置诊断信标：页面每次加载、每次点菜单都会向板子汇报，
板子从**串口打印**。排查"手机上和电脑上不一样"时，一边操作一边看串口：

```
[BP-DIAG] diag=1&t=load2&len=29326&modal=1&links=8&err=
[BP-DIAG] diag=1&t=click:舵机校准:cal
[CMD] get_cal=1&_bp=1793189351031
```

| 看到什么 | 结论 |
|---|---|
| 完全没有 `[BP-DIAG]` | 我们那段脚本在手机上没运行（旧缓存页 / 页面被截断 / 别的问题） |
| 有 `t=click:舵机校准:cal` | 点击已被捕获并正确解析，问题在别处 |
| 有 `t=click:...:none` | 菜单项没解析出动作（菜单文字被改过） |
| `err=` 后面有内容 | 页面有 JS 报错，内容就是错误信息 |

---

## 七、服务端根因修复（已硬件回归）

### 病根在固件里

`frozen/camera_stream.py` 的 `_accept_loop()`：

```python
client.settimeout(2.0)
raw = client.recv(512)        # 只读一次，最多 512 字节
```

服务器是**单线程**的，所以：

- 手机浏览器的预连接、被取消的请求（连上但不发完请求）会把**整个服务阻塞满 2 秒**
  （实测：正常 144ms → 2119ms）。页面于是慢慢吐出，用户点到还没有处理函数的按钮；
- 超过 512 字节或分多段到达的请求只会被读到一部分。

### 修复方式（无需重烧固件）

`start()` 是**通过模块全局名**启动线程的：

```python
_running = True
_thread.start_new_thread(_accept_loop, ())
```

因此在 `camera_on.py` 里于 `camera_stream.start()` **之前**替换
`camera_stream._accept_loop` 即可生效，路由与图传逻辑与原版逐行一致：

| 项 | 原版 | 本版 |
|---|---|---|
| 连接超时 | 2.0 s | **0.5 s**（另有 2 s 总时长上限，防慢速 Trickle） |
| 读请求 | 单次 `recv(512)` | **循环读到 `\r\n\r\n` 完整为止**（上限 1024 字节） |
| 路由 `/stream` `/cmd` 其它 | — | 与原版一致（含 `_stream_threads` 计数） |

### 硬件回归结果（2026-10-28）

| 测试 | 结果 |
|---|---|
| 页面 / `get_cal` / `sys_info` / `get_params` / `heading=state` | 全部正常（8~206ms） |
| **半开连接下的请求延迟** | **406ms**（修复前 2119ms） |
| 超过 512 字节的长 URL | 正常（修复前会被截断） |
| 请求分两段到达 | 正常（修复前失败） |
| 慢速客户端（512B/20ms）读全页 | 31525 字节不丢 |
| 连续 15 次请求 | 15/15 |
| `/stream` 路由（无摄像头） | 正确回 404 |
| 页面字节数 | 与修复前完全一致（30049） |

> 未测项：**带摄像头的图传**（`_send_stream` 路径）—— 代码与原版一致，但手边没有
> 摄像头模组。用图传的同学请额外确认一下。

### 建议的固件侧同步修改

`camera_on.py` 的替换只是权宜；根子还在 `frozen/camera_stream.py`，建议同步改掉
（改固件需要重新编译烧录，代价更高，所以先用开关文件顶着）：

```python
# frozen/camera_stream.py  _accept_loop()
client.settimeout(0.5)                 # 原 2.0
raw = b""
while b"\r\n\r\n" not in raw and len(raw) < 1024:
    chunk = client.recv(256)
    if not chunk:
        break
    raw += chunk
```

---

## 八、其他说明

- **「引导」弹窗**：板子开了强制门户（DNS 劫持 53 端口），对任意网址都返回同一个
  页面；手机系统据此弹一次"登录网络"迷你浏览器。**弹不弹由手机系统决定**，
  关掉后不再弹是正常现象（系统已记住该网络）。想再看：WiFi 设置里点该网络重新登录，
  或"忘记网络"重连。板子侧无法控制；如需每次打开页面都有引导，可在页面里加
  说明层（当前未做）。
- **舵机零偏**存放在 NVS，断电不丢失；每次调整立即写 Flash。
- 没有 IMU 的板子，IMU 校准会明确提示「未检测到 IMU，请检查接线」，
  不会假装成功。

> ⚠️ **不要在应用层另建 I2C 对象扫总线找 IMU** —— 固件已占用 I2C0，再
> `I2C(0, scl=Pin(21), sda=Pin(14))` + `scan()` 会把板子**硬复位**
> （实测 `reset_cause()==2`，运行几十秒后突然重启，热点/图传全断）。
> 查 IMU 只能问驱动：`bpuppy_imu.init(0,14,21,0x68)` → `is_ready()` / `get_chip()`。
> 详见 AGENTS.md 易错点 0。
