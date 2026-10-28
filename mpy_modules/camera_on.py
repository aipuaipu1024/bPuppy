# /camera_on.py -- bPuppy web debug UI (enhanced boot switch, with server fix).
#
# 这是上电开关文件的**增强版**：在保留原厂遥控页的前提下，把顶部「设置」菜单里的
# 6 个面板（舵机校准 / IMU 校准 / 运动参数 / 持久化参数 / 系统信息 / 语言设置）
# 做成可用的功能界面，并修掉若干底层问题。
#
# 出厂版只有两行（见 docs/wifi设备遥控指南.md 2.3）：
#     import camera_stream
#     camera_stream.start(stream=True)
# 想要出厂行为，把本文件替换成那两行即可 —— 文件名本身就是开关，下次上电生效。
#
# ⚠️ 必须用 USB + mpremote 传输：文件内含中文界面文案，走蓝牙/ViperIDE 会丢字节。
#
# 四个关键点（详细分析见 docs/web调试界面修复说明.md）：
#   1. HTTP 响应用 camera_stream._html_page() 生成，改 _HTML_TEMPLATE 无效，
#      所以这里包装 _html_page()，把 UI 注入真实响应体；
#   2. 固件用 exec(代码, globals()) 把本文件与 /pwm_ext_on.py、/main.py、REPL、
#      mpremote run 脚本塞进同一命名空间 —— 任何一处 `s = ...` 都会覆盖舵机模块。
#      因此本补丁全部用闭包 + 函数内 import，不依赖可被覆盖的全局名；
#   3. 菜单按钮改用捕获阶段事件委托（文档一解析出来就能用），不再依赖文档最底部
#      那段脚本；
#   4. 替换固件的 _accept_loop：原版 client.settimeout(2.0) + 单次 recv(512)，
#      手机浏览器的预连接/取消请求会把单线程服务器阻塞满 2 秒（实测 144ms→2119ms），
#      页面慢慢吐出，提前点到按钮就没反应。改为 0.5s 超时 + 读到请求头完整为止
#      （实测阻塞降到 406ms，且支持长 URL 与分包请求）。start() 通过模块全局名
#      启动线程，所以在 start() 前替换即可生效，路由与图传逻辑与原版一致。

import camera_stream


# ---------------------------------------------------------------------------
# 1. Extra /cmd endpoints used by the debug UI
# ---------------------------------------------------------------------------
# NOTE: the firmware exec()s this file into ONE namespace shared with
# /pwm_ext_on.py, /main.py (the KittenBlock program), the serial REPL and
# anything executed with `mpremote run`. A variable named `s` (or `page`,
# `get`, `cal`, ...) in any of those silently shadows what this file needs --
# that is how servo calibration once died with
# "ERR:'socket' object has no attribute 'get_cal_point'".
# So: closures + imports inside the function body. Neither can be shadowed.
def _make_bp_parse_cmd(orig_parse_cmd):
    def _bp_parse_cmd(qs):
        import bpuppy_servo as _sv
        try:
            # diagnostics beacon from the page: printed on the serial console
            if 'diag=' in qs:
                print('[BP-DIAG]', qs[:220])
                return 'OK'
            # log everything except the 1.5s speed poll, so a failing phone can
            # be traced from the serial console
            if 'get_params=1' not in qs:
                print('[CMD]', qs[:90])

            if 'get_cal=1' in qs:
                return ','.join('%.1f' % _sv.get_cal_point(i, 1) for i in range(8))

            if 'cal=1' in qs:
                params = dict(kv.split('=') for kv in qs.split('&') if '=' in kv)
                ch = int(params.get('ch', 0))
                deg = float(params.get('deg', 90.0))
                _sv.cal_point(ch, 1, deg)
                return 'OK:cal'

            if 'all90=1' in qs:
                import time
                for ch in range(8):
                    _sv.set_angle(ch, 90.0)
                    # stagger the moves: 8 servos starting at the same instant
                    # is a current spike that browns out USB-only powered boards
                    time.sleep_ms(40)
                return 'OK:all90'

            if 'balance=toggle' in qs:
                import balance
                if getattr(balance, '_running', False):
                    balance.stop()
                    return '自平衡已关闭'
                balance.start()
                return '自平衡已开启'

            if 'heading=off' in qs:
                import heading_follow
                heading_follow.stop()
                return '指南小狗已关闭'

            if 'heading=on' in qs:
                import heading_follow
                params = dict(kv.split('=') for kv in qs.split('&') if '=' in kv)
                deg = float(params.get('deg', 0))
                heading_follow.start(deg)
                return '指南小狗已开启（目标 %d°）' % int(deg)

            if 'heading=state' in qs:
                import heading_follow
                _run = getattr(heading_follow, 'running', None)
                return 'ON' if (_run() if callable(_run) else bool(_run)) else 'OFF'

            if 'sys_info=1' in qs:
                import voltage
                import gc
                return '{"v":%.2f,"pct":%d,"ram":%d}' % (
                    voltage.read_v(), voltage.read_pct(), gc.mem_free())

            if 'cal_imu=1' in qs:
                import bpuppy_imu as imu
                imu.init(0, 14, 21, 0x68)
                if not imu.is_ready():
                    return 'ERR:未检测到 IMU，请检查接线'
                imu.calibrate(300)
                return 'IMU 校准完成，已写入 NVS'

        except Exception as e:
            return 'ERR:' + str(e)

        return orig_parse_cmd(qs)

    return _bp_parse_cmd


camera_stream._parse_cmd = _make_bp_parse_cmd(camera_stream._parse_cmd)

# ---------------------------------------------------------------------------
# 2. UI fragment: styles + panels + script, injected before </body>
# ---------------------------------------------------------------------------
BP_UI = r'''
<style id="bp-style">
.bp-modal{display:none;position:fixed;top:0;left:0;right:0;bottom:0;z-index:99999;background:#0f1115;overflow-y:auto;-webkit-overflow-scrolling:touch;padding:14px;color:#e5e7eb;font-family:system-ui,-apple-system,sans-serif}
.bp-modal.bp-open{display:block}
.bp-inner{max-width:560px;margin:0 auto;padding-bottom:40px}
.bp-head{position:sticky;top:0;z-index:2;display:flex;justify-content:space-between;align-items:center;gap:10px;padding:10px 0 12px;margin-bottom:12px;background:#0f1115;border-bottom:1px solid #2a2f3a}
.bp-title{font-size:17px;font-weight:700}
.bp-close{background:#ef4444;color:#fff;border:none;border-radius:9px;padding:10px 14px;font-size:13px;font-weight:700;cursor:pointer;touch-action:manipulation}
.bp-actions{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:12px}
.bp-act{color:#fff;border:none;border-radius:9px;padding:12px 8px;font-size:13px;font-weight:700;cursor:pointer;touch-action:manipulation}
.bp-blue{background:#3b82f6}.bp-green{background:#10b981}.bp-purple{background:#8b5cf6}.bp-gray{background:#6b7280}
.bp-card{background:#1f2937;border:1px solid #374151;border-radius:11px;padding:11px 12px;margin-bottom:9px}
.bp-row{display:flex;justify-content:space-between;align-items:center;gap:10px;font-size:14px;margin-bottom:7px}
.bp-row b{color:#60a5fa}
.bp-ctrl{display:flex;align-items:center;gap:6px}
.bp-step{flex:0 0 48px;height:40px;background:#374151;color:#fff;border:none;border-radius:7px;font-weight:700;font-size:13px;cursor:pointer;touch-action:manipulation}
.bp-step:active{background:#4b5563}
.bp-range{flex:1;height:30px;accent-color:#3b82f6}
.bp-hint{color:#9ca3af;font-size:12px;line-height:1.6;margin-top:12px}
.bp-p{color:#9ca3af;font-size:14px;line-height:1.7;margin-bottom:16px}
.bp-big{display:block;width:100%;background:#2563eb;color:#fff;border:none;border-radius:10px;padding:14px;font-size:15px;font-weight:700;cursor:pointer;touch-action:manipulation}
.bp-big:disabled{opacity:.6}
.bp-status{margin-top:14px;color:#60a5fa;font-size:13px;line-height:1.6;min-height:20px}
.bp-lang-active{border-color:#10b981;display:flex;justify-content:space-between}
.bp-lang-disabled{opacity:.5;display:flex;justify-content:space-between}
.bp-toast{position:fixed;left:50%;bottom:26px;transform:translateX(-50%);background:rgba(0,0,0,.92);color:#fff;padding:10px 18px;border-radius:24px;font-size:14px;font-weight:600;opacity:0;pointer-events:none;transition:opacity .2s;z-index:100000;max-width:90vw;text-align:center}
.bp-toast.bp-show{opacity:1}
body.bp-lock{overflow:hidden}
</style>

<div id="bpToast" class="bp-toast"></div>

<div id="bpCalModal" class="bp-modal">
  <div class="bp-inner">
    <div class="bp-head"><div class="bp-title">🎯 8 路舵机角度校准台</div><button class="bp-close" data-close="bpCalModal">✕ 完成</button></div>
    <div class="bp-actions">
      <button class="bp-act bp-blue" data-cmd="all90=1" data-msg="📐 全部归 90°">📐 全部归 90°</button>
      <button class="bp-act bp-green" data-cmd="pose=stand" data-msg="🐾 站立测试">🐾 站立测试</button>
      <button class="bp-act bp-purple" data-cmd="gait=sit" data-msg="🧎 坐下测试">🧎 坐下测试</button>
      <button class="bp-act bp-gray" data-cmd="gait=stop" data-msg="🛑 释放舵机">🛑 释放舵机</button>
    </div>
    <div id="bpCalCards"></div>
    <div class="bp-hint">拖动滑块或点按 ± 按钮微调；每次调整会立即写入 Flash。</div>
  </div>
</div>

<div id="bpImuModal" class="bp-modal">
  <div class="bp-inner">
    <div class="bp-head"><div class="bp-title">🧭 IMU 姿态零偏校准</div><button class="bp-close" data-close="bpImuModal">✕ 完成</button></div>
    <p class="bp-p">请将小狗放在绝对水平的桌面上，保持静止，然后点击下方按钮。采样约需 3 秒，期间不要碰它；若板端检测不到 IMU 会直接报错。</p>
    <button class="bp-big" data-imu="1">开始水平校准并写入 NVS</button>
    <div id="bpImuStatus" class="bp-status"></div>
  </div>
</div>

<div id="bpMotionModal" class="bp-modal">
  <div class="bp-inner">
    <div class="bp-head"><div class="bp-title">⚙️ 运动参数</div><button class="bp-close" data-close="bpMotionModal">✕ 完成</button></div>
    <div class="bp-card">
      <div class="bp-row"><span>速度 (Speed)</span><b id="bpSpeedVal">2.5</b></div>
      <input id="bpSpeed" class="bp-range" type="range" min="0" max="10" step="0.5" value="2.5" data-param="speed">
    </div>
    <div class="bp-card">
      <div class="bp-row"><span>步幅 (Stride)</span><b id="bpStrideVal">70 mm</b></div>
      <input id="bpStride" class="bp-range" type="range" min="30" max="110" step="5" value="70" data-param="stride">
    </div>
    <div class="bp-card">
      <div class="bp-row"><span>站立高度 (Height)</span><b id="bpHeightVal">70 mm</b></div>
      <input id="bpHeight" class="bp-range" type="range" min="50" max="90" step="2" value="70" data-param="height">
    </div>
    <div class="bp-hint">参数会立即下发；若组合超出安全范围，板端会保持原值。</div>
  </div>
</div>

<div id="bpNvsModal" class="bp-modal">
  <div class="bp-inner">
    <div class="bp-head"><div class="bp-title">💾 持久化参数</div><button class="bp-close" data-close="bpNvsModal">✕ 完成</button></div>
    <div id="bpNvsList"></div>
    <div class="bp-hint">这些是保存在 Flash NVS 中的 8 路零偏角度，断电不丢失。</div>
  </div>
</div>

<div id="bpSysModal" class="bp-modal">
  <div class="bp-inner">
    <div class="bp-head"><div class="bp-title">ℹ️ 系统信息</div><button class="bp-close" data-close="bpSysModal">✕ 完成</button></div>
    <div class="bp-card"><div class="bp-row"><span>主控芯片</span><b>ESP32-S3 (240MHz 双核)</b></div></div>
    <div class="bp-card"><div class="bp-row"><span>存储容量</span><b>16 MB Flash + 8 MB PSRAM</b></div></div>
    <div class="bp-card"><div class="bp-row"><span>电池状态</span><b id="bpSysBatt">检测中...</b></div></div>
    <div class="bp-card"><div class="bp-row"><span>空闲内存</span><b id="bpSysRam">--</b></div></div>
    <div class="bp-card"><div class="bp-row"><span>固件版本</span><b>bPuppy v3.0 (IDF v5.1.2)</b></div></div>
  </div>
</div>

<div id="bpLangModal" class="bp-modal">
  <div class="bp-inner">
    <div class="bp-head"><div class="bp-title">🌐 语言设置</div><button class="bp-close" data-close="bpLangModal">✕ 完成</button></div>
    <div class="bp-card bp-lang-active"><span>简体中文</span><b>✓ 当前</b></div>
    <div class="bp-card bp-lang-disabled"><span>English</span><b>暂未支持</b></div>
    <div class="bp-hint">当前固件仅内置简体中文界面。</div>
  </div>
</div>

<div id="bpHeadingModal" class="bp-modal">
  <div class="bp-inner">
    <div class="bp-head"><div class="bp-title">🧭 指南小狗</div><button class="bp-close" data-close="bpHeadingModal">✕ 完成</button></div>
    <p class="bp-p">让小狗保持指向固定航向。选好目标航向后点击开始；此功能依赖板载 IMU。</p>
    <div class="bp-card">
      <div class="bp-row"><span>目标航向</span><b id="bpHeadVal">0°</b></div>
      <input id="bpHead" class="bp-range" type="range" min="0" max="359" step="1" value="0" data-heading="1">
    </div>
    <div class="bp-actions">
      <button class="bp-act bp-green" data-hon="1">▶ 开始</button>
      <button class="bp-act bp-gray" data-hoff="1">⏹ 停止</button>
    </div>
    <div id="bpHeadStatus" class="bp-status"></div>
    <div class="bp-hint">航向以磁北为 0°，顺时针增加。没有 IMU 的板子会直接报错。</div>
  </div>
</div>

<script id="bp-script">
(function(){
  var JOINTS = [["左前大腿","LF_HIP (CH0)"],["左前小腿","LF_KNEE (CH1)"],["左后大腿","LH_HIP (CH2)"],["左后小腿","LH_KNEE (CH3)"],["右前大腿","RF_HIP (CH4)"],["右前小腿","RF_KNEE (CH5)"],["右后大腿","RH_HIP (CH6)"],["右后小腿","RH_KNEE (CH7)"]];
  var calAngles = [90,90,90,90,90,90,90,90];
  var imuBusy = false;

  function $(id){ return document.getElementById(id); }
  // fetch() only rejects on network errors, NOT on HTTP error statuses -- a
  // captive portal / dead gateway answers 502 and would look like success.
  function bpGet(q){
    // unique query param: /cmd answers must never be served from cache
    return fetch('/cmd?' + q + '&_bp=' + Date.now()).then(function(r){
      if(!r.ok) throw new Error('HTTP ' + r.status);
      return r.text();
    });
  }
  function closest(el, sel){
    while(el && el !== document){
      if(el.matches && el.matches(sel)) return el;
      el = el.parentNode;
    }
    return null;
  }
  function toast(m){
    var t = $('bpToast');
    if(!t) return;
    t.textContent = m;
    t.classList.add('bp-show');
    clearTimeout(t._bpTimer);
    t._bpTimer = setTimeout(function(){ t.classList.remove('bp-show'); }, 2200);
  }
  function closeMenus(){
    var ms = document.querySelectorAll('.menu.open');
    for(var i=0;i<ms.length;i++) ms[i].classList.remove('open');
  }
  function openModal(id){
    closeMenus();
    var m = $(id);
    if(!m) return;
    m.classList.add('bp-open');
    document.body.classList.add('bp-lock');
    if(id === 'bpCalModal') loadCal();
    if(id === 'bpNvsModal') loadNvs();
    if(id === 'bpSysModal') loadSys();
    if(id === 'bpMotionModal') loadParams();
    if(id === 'bpHeadingModal') loadHeading();
  }
  function closeModal(id){
    var m = $(id);
    if(m) m.classList.remove('bp-open');
    document.body.classList.remove('bp-lock');
  }
  function api(q, msg){
    return bpGet(q).then(function(t){
      t = (t || '').trim();
      if(t.indexOf('ERR:') === 0) toast(t);
      else toast(msg || t);
      return t;
    }).catch(function(){ toast('⚠ 机器狗无响应，热点可能已断开，请刷新页面'); });
  }
  function offline(msg){
    return '⚠ 机器狗无响应，热点可能已断开。请确认手机仍连着 bPuppy_AE25，然后刷新页面。';
  }

  // Resolve a dropdown link to an action AT CLICK TIME. Nothing is pre-bound,
  // so it does not matter when this script runs relative to the menu markup,
  // and a rewording of the labels can never leave an item dead again.
  function bpAction(a){
    var attr = a.getAttribute('data-bp');
    if(attr) return attr;
    var txt = (a.textContent || '').replace(/\s+/g, '');
    if(txt.indexOf('舵机校准') >= 0) return 'cal';
    if(txt.indexOf('IMU') >= 0) return 'imu';
    if(txt.indexOf('运动参数') >= 0) return 'motion';
    if(txt.indexOf('持久化参数') >= 0) return 'nvs';
    if(txt.indexOf('系统信息') >= 0) return 'sys';
    if(txt.indexOf('语言设置') >= 0) return 'lang';
    if(txt.indexOf('指南小狗') >= 0) return 'heading';
    if(txt.indexOf('自平衡') >= 0) return 'balance';
    // positional fallback inside its own dropdown
    var menu = closest(a, '.menu');
    if(menu){
      var btn = menu.querySelector('.menu-btn');
      var label = btn ? (btn.textContent || '') : '';
      var order = null;
      if(label.indexOf('设置') >= 0) order = ['cal','imu','motion','nvs','sys','lang'];
      else if(label.indexOf('功能') >= 0) order = ['heading','balance'];
      if(order){
        var as = menu.querySelectorAll('.dropdown a');
        for(var i=0;i<as.length;i++){ if(as[i] === a) return order[i] || null; }
      }
    }
    return null;
  }

  // --- diagnostics: report to the board over serial so a failing phone can be
  // --- debugged without guessing. Everything is printed by /cmd?diag=...
  var bpErrs = [];
  window.addEventListener('error', function(e){
    bpErrs.push(String(e.message || e).slice(0, 60));
  });
  function bpBeacon(extra){
    try{
      var q = 'diag=1&t=' + (extra || 'x')
        + '&href=' + encodeURIComponent(String(location.href).slice(-40))
        + '&len=' + document.documentElement.innerHTML.length
        + '&modal=' + (document.getElementById('bpCalModal') ? 1 : 0)
        + '&links=' + document.querySelectorAll('.dropdown a').length
        + '&err=' + encodeURIComponent(bpErrs.join('|').slice(0, 90));
      fetch('/cmd?' + q).catch(function(){});
    }catch(e){}
  }
  bpBeacon('load');
  setTimeout(function(){ bpBeacon('load2'); }, 1000);

  document.addEventListener('click', function(e){
    // tapping the dark backdrop closes the panel, so it can never feel stuck
    if(e.target && e.target.classList && e.target.classList.contains('bp-modal')){
      e.preventDefault();
      closeModal(e.target.id);
      return;
    }
    // Menu buttons: handled here (capture phase) so the dropdown works the
    // instant the markup is parsed. The page's own handler sits at the very
    // bottom of the document, so on a slow load a tap used to hit a button
    // with no handler at all. stopPropagation keeps the two from fighting.
    var mb = closest(e.target, '.menu-btn');
    if(mb){
      e.preventDefault(); e.stopPropagation();
      var menu = closest(mb, '.menu');
      if(menu){
        var wasOpen = menu.classList.contains('open');
        closeMenus();
        if(!wasOpen) menu.classList.add('open');
      }
      return;
    }
    var a = closest(e.target, '.dropdown a');
    if(a){
      e.preventDefault(); e.stopPropagation();
      closeMenus();
      var act = bpAction(a);
      bpBeacon('click:' + encodeURIComponent((a.textContent || '').trim().slice(0, 12)) + ':' + (act || 'none'));
      if(act === 'cal') openModal('bpCalModal');
      else if(act === 'imu') openModal('bpImuModal');
      else if(act === 'motion') openModal('bpMotionModal');
      else if(act === 'nvs') openModal('bpNvsModal');
      else if(act === 'sys') openModal('bpSysModal');
      else if(act === 'lang') openModal('bpLangModal');
      else if(act === 'heading') openModal('bpHeadingModal');
      else if(act === 'balance') api('balance=toggle');
      return;
    }
    var b = closest(e.target, 'button[data-cmd]');
    if(b){
      e.preventDefault(); e.stopPropagation();
      var cmd = b.getAttribute('data-cmd');
      var p = api(cmd, b.getAttribute('data-msg'));
      if(cmd === 'all90=1'){
        p.then(function(t){
          if(t && t.indexOf('ERR:') !== 0){
            for(var k=0;k<8;k++) calAngles[k] = 90;
            renderCal();
          }
        });
      }
      return;
    }
    var c = closest(e.target, 'button[data-close]');
    if(c){
      e.preventDefault(); e.stopPropagation();
      closeModal(c.getAttribute('data-close'));
      return;
    }
    var imu = closest(e.target, 'button[data-imu]');
    if(imu){
      e.preventDefault(); e.stopPropagation();
      startImu();
      return;
    }
    var step = closest(e.target, '#bpCalCards button[data-ch]');
    if(step){
      e.preventDefault(); e.stopPropagation();
      stepCal(parseInt(step.getAttribute('data-ch'),10), parseFloat(step.getAttribute('data-delta')));
      return;
    }
    var hon = closest(e.target, 'button[data-hon]');
    if(hon){
      e.preventDefault(); e.stopPropagation();
      var deg = parseInt($('bpHead').value, 10);
      api('heading=on&deg=' + deg, '指南小狗已开启').then(function(t){
        if(t && t.indexOf('ERR:') !== 0) loadHeading();
      });
      return;
    }
    var hoff = closest(e.target, 'button[data-hoff]');
    if(hoff){
      e.preventDefault(); e.stopPropagation();
      api('heading=off', '指南小狗已关闭').then(function(t){
        if(t && t.indexOf('ERR:') !== 0) loadHeading();
      });
      return;
    }
  }, true);

  document.addEventListener('pointerdown', function(e){
    if(!closest(e.target, '.menu')) closeMenus();
  });

  function loadCal(){
    bpGet('get_cal=1').then(function(t){
      var parts = (t || '').trim().split(',');
      if(parts.length === 8){
        for(var i=0;i<8;i++){
          var v = parseFloat(parts[i]);
          if(!isNaN(v)) calAngles[i] = v;
        }
      }
      renderCal();
    }).catch(function(){ renderCal(); offline('cal'); });
  }
  function renderCal(){
    var h = '';
    for(var i=0;i<8;i++){
      h += '<div class="bp-card">';
      h += '<div class="bp-row"><div><b>' + JOINTS[i][0] + '</b> <span style="color:#9ca3af;font-size:11px">' + JOINTS[i][1] + '</span></div><b id="bpv' + i + '">' + calAngles[i].toFixed(1) + '°</b></div>';
      h += '<div class="bp-ctrl">';
      h += '<button class="bp-step" data-ch="' + i + '" data-delta="-1">-1°</button>';
      h += '<button class="bp-step" data-ch="' + i + '" data-delta="-0.5">-.5°</button>';
      h += '<input class="bp-range" type="range" data-ch="' + i + '" min="60" max="120" step="0.5" value="' + calAngles[i] + '">';
      h += '<button class="bp-step" data-ch="' + i + '" data-delta="0.5">+.5°</button>';
      h += '<button class="bp-step" data-ch="' + i + '" data-delta="1">+1°</button>';
      h += '</div></div>';
    }
    $('bpCalCards').innerHTML = h;
  }
  function setCal(ch, val){
    val = Math.max(60, Math.min(120, parseFloat(val)));
    if(isNaN(val)) return;
    calAngles[ch] = val;
    var v = $('bpv' + ch);
    if(v) v.textContent = val.toFixed(1) + '°';
    var r = document.querySelector('input[data-ch="' + ch + '"]');
    if(r) r.value = val;
    api('cal=1&ch=' + ch + '&deg=' + val.toFixed(1), '✓ CH' + ch + ' 设为 ' + val.toFixed(1) + '°（已存 Flash）');
  }
  function stepCal(ch, delta){
    setCal(ch, calAngles[ch] + delta);
  }

  document.addEventListener('input', function(e){
    var r = e.target;
    if(!r || r.type !== 'range') return;
    if(r.hasAttribute('data-ch')){
      var ch = parseInt(r.getAttribute('data-ch'),10);
      var v = $('bpv' + ch);
      if(v) v.textContent = parseFloat(r.value).toFixed(1) + '°';
    } else if(r.hasAttribute('data-param')){
      var p = r.getAttribute('data-param');
      if(p === 'speed') $('bpSpeedVal').textContent = parseFloat(r.value).toFixed(1);
      else if(p === 'stride') $('bpStrideVal').textContent = parseFloat(r.value).toFixed(0) + ' mm';
      else if(p === 'height') $('bpHeightVal').textContent = parseFloat(r.value).toFixed(0) + ' mm';
    } else if(r.hasAttribute('data-heading')){
      $('bpHeadVal').textContent = parseInt(r.value, 10) + '°';
    }
  });
  document.addEventListener('change', function(e){
    var r = e.target;
    if(!r || r.type !== 'range') return;
    if(r.hasAttribute('data-ch')){
      setCal(parseInt(r.getAttribute('data-ch'),10), r.value);
    } else if(r.hasAttribute('data-param')){
      var p = r.getAttribute('data-param');
      if(p === 'speed') api('set=1&speed=' + r.value, '速度设为 ' + r.value);
      else if(p === 'stride') api('stride=' + r.value, '步幅设为 ' + r.value + ' mm');
      else if(p === 'height') api('height=' + r.value, '站立高度设为 ' + r.value + ' mm');
    }
  });

  function loadNvs(){
    bpGet('get_cal=1').then(function(t){
      var arr = (t || '').trim().split(',');
      var h = '';
      for(var i=0;i<8;i++){
        h += '<div class="bp-card"><div class="bp-row"><span>' + JOINTS[i][0] + ' <span style="color:#9ca3af;font-size:11px">' + JOINTS[i][1] + '</span></span><b>' + (arr[i] || '90.0') + '°</b></div></div>';
      }
      $('bpNvsList').innerHTML = h;
    }).catch(function(){ $('bpNvsList').innerHTML = '<div class="bp-card"><div class="bp-row"><span>' + offline() + '</span></div></div>'; });
  }

  function loadSys(){
    bpGet('sys_info=1').then(function(t){
      try {
        var d = JSON.parse(t);
        $('bpSysBatt').textContent = Number(d.v).toFixed(2) + ' V (' + d.pct + '%)';
        $('bpSysRam').textContent = Math.round(Number(d.ram)/1024) + ' KB';
      } catch(e) {
        $('bpSysBatt').textContent = '读取失败';
      }
    }).catch(function(){ $('bpSysBatt').textContent = '读取失败'; $('bpSysRam').textContent = offline(); });
  }

  function loadParams(){
    bpGet('get_params=1').then(function(t){
      var m = {};
      (t || '').split('&').forEach(function(kv){
        var p = kv.split('=');
        if(p.length === 2) m[p[0]] = parseFloat(p[1]);
      });
      if(m.SPEED !== undefined){ $('bpSpeed').value = m.SPEED; $('bpSpeedVal').textContent = m.SPEED.toFixed(1); }
      if(m.STRIDE !== undefined){ $('bpStride').value = m.STRIDE; $('bpStrideVal').textContent = m.STRIDE.toFixed(0) + ' mm'; }
      if(m.HEIGHT !== undefined){ $('bpHeight').value = m.HEIGHT; $('bpHeightVal').textContent = m.HEIGHT.toFixed(0) + ' mm'; }
    }).catch(function(){ toast(offline()); });
  }

  function loadHeading(){
    bpGet('heading=state').then(function(t){
      var on = (t || '').trim().toUpperCase() === 'ON';
      $('bpHeadStatus').textContent = on ? '运行中：正在保持当前目标航向' : '未运行';
    }).catch(function(){ $('bpHeadStatus').textContent = offline(); });
  }

  function startImu(){
    if(imuBusy) return;
    imuBusy = true;
    var b = document.querySelector('button[data-imu]');
    if(b) b.disabled = true;
    $('bpImuStatus').textContent = '正在采样校准中，请勿晃动小狗...';
    bpGet('cal_imu=1').then(function(t){
      $('bpImuStatus').textContent = t;
      toast(t);
    }).catch(function(){
      $('bpImuStatus').textContent = offline();
    }).then(function(){
      imuBusy = false;
      if(b) b.disabled = false;
    });
  }
  // Connection watchdog. The AP can drop (power cycle, brownout, reboot) and
  // the page then looks "dead" with no explanation. Say so, loudly.
  var bpFails = 0;
  var bpBanner = null;
  function bpWatch(){
    bpGet('sys_info=1').then(function(){
      bpFails = 0;
      if(bpBanner) bpBanner.style.display = 'none';
    }).catch(function(){
      bpFails++;
      if(bpFails < 2) return;
      if(!bpBanner){
        bpBanner = document.createElement('div');
        bpBanner.style.cssText = 'position:fixed;left:0;right:0;bottom:0;background:#b91c1c;color:#fff;text-align:center;font-size:13px;font-weight:700;padding:9px 12px;z-index:100001;font-family:system-ui,sans-serif';
        document.body.appendChild(bpBanner);
      }
      bpBanner.textContent = '⚠ 机器狗无响应：热点断开或板子正在重启。等它启动完（约 15 秒）再刷新页面';
      bpBanner.style.display = 'block';
    });
  }
  bpWatch();
  setInterval(bpWatch, 10000);
})();
</script>
'''

# ---------------------------------------------------------------------------
# 3. Inject the UI into the page that is actually served
# ---------------------------------------------------------------------------
_BP_CACHE_META = ('<meta http-equiv="Cache-Control" content="no-store, no-cache, must-revalidate">'
                  '<meta http-equiv="Pragma" content="no-cache">'
                  '<meta http-equiv="Expires" content="0">')


def _make_bp_html_page(orig_html_page, ui, cache_meta):
    def _bp_html_page(*args, **kwargs):
        html = orig_html_page(*args, **kwargs)
        if 'id="bpCalModal"' not in html:
            # Inject right after <body>, NOT before </body>. Everything the UI
            # needs is fixed-position and resolves its targets at click time,
            # so it works even if the tail of the document never arrives.
            if '<body>' in html:
                html = html.replace('<body>', '<body>' + ui, 1)
            elif '</body>' in html:
                html = html.replace('</body>', ui + '</body>', 1)
            else:
                html = html + ui
        # Phones cache this page aggressively (the firmware sends no cache
        # headers), which is how a stale page with the old broken script keeps
        # being served. Force a fresh fetch every time.
        if 'Cache-Control' not in html:
            cut = html.find('\r\n\r\n')
            if cut >= 0:
                html = (html[:cut]
                        + '\r\nCache-Control: no-store, no-cache, must-revalidate'
                        + '\r\nPragma: no-cache'
                        + '\r\nExpires: 0'
                        + html[cut:])
            if '<head>' in html:
                html = html.replace('<head>', '<head>' + cache_meta, 1)
        # every request costs the board a fresh TCP connection, and a phone
        # that polls hard is what keeps the single-threaded server busy
        html = html.replace('setInterval(syncSpeed, 1500)', 'setInterval(syncSpeed, 5000)')
        return html

    return _bp_html_page


camera_stream._html_page = _make_bp_html_page(camera_stream._html_page, BP_UI, _BP_CACHE_META)

# ---------------------------------------------------------------------------
# 4. Replace the firmware's accept loop, then start the server
# ---------------------------------------------------------------------------
# Root cause of the "sometimes a tap does nothing" behaviour, from
# frozen/camera_stream.py:
#
#     client.settimeout(2.0)
#     raw = client.recv(512)          # ONE read of at most 512 bytes
#
# The server is single threaded, so:
#   * a phone connection that never finishes its request (preconnect, an
#     aborted fetch) blocks every other request for the full 2 s -- measured
#     144 ms -> 2119 ms. The page then trickles in and the user taps the menu
#     before the page's own script (at the very bottom of the document) has
#     run, so the tap hits a button with no handler at all.
#   * a request split over several TCP segments, or longer than 512 bytes,
#     is read only partially.
#
# start() launches the loop through the module global, so replacing it here
# (before start()) is enough. Routing and streaming are unchanged.
def _bp_accept_loop():
    import socket as _sock
    import time as _time
    import camera_stream as _cs

    srv = _sock.socket()
    srv.setsockopt(_sock.SOL_SOCKET, _sock.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", 80))
    srv.listen(3)

    while _cs._running:
        try:
            srv.settimeout(1.0)
            client, addr = srv.accept()
        except OSError:
            continue

        # read the whole request header: shorter timeout, and keep reading
        # until it is complete instead of trusting a single 512 byte recv
        try:
            client.settimeout(0.5)
            deadline = _time.ticks_add(_time.ticks_ms(), 2000)
            raw = b""
            while (b"\r\n\r\n" not in raw) and len(raw) < 1024:
                if _time.ticks_diff(deadline, _time.ticks_ms()) <= 0:
                    break
                chunk = client.recv(256)
                if not chunk:
                    break
                raw += chunk
        except OSError:
            client.close()
            continue

        try:
            request = raw.decode("utf-8", "ignore")
        except UnicodeError:
            request = ""

        if not request:
            client.close()
            continue

        lines = request.split("\r\n")
        if not lines:
            client.close()
            continue

        first_line = lines[0]

        if "/stream" in first_line:
            if _cs._stream_on and _cs._thread:
                _cs._stream_threads = _cs._stream_threads + 1
                try:
                    _cs._thread.start_new_thread(_cs._send_stream, (client,))
                except OSError:
                    _cs._stream_threads = _cs._stream_threads - 1
                    client.close()
            else:
                try:
                    client.send(b"HTTP/1.0 404 Not Found\r\n\r\nstream off")
                except OSError:
                    pass
                client.close()

        elif "/cmd" in first_line:
            try:
                path = first_line.split(" ")[1]
                result = _cs._parse_cmd(path)
                client.send(("HTTP/1.0 200 OK\r\n\r\n" + result).encode("utf-8"))
            except Exception as e:
                print("CMD ERR:", e)
            client.close()

        else:
            _cs._send_html(client)

    srv.close()


camera_stream._accept_loop = _bp_accept_loop

# ---------------------------------------------------------------------------
# 5. Start the original web server (AP + HTTP + camera stream button)
# ---------------------------------------------------------------------------
camera_stream.start()
