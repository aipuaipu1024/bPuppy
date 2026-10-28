#!/usr/bin/env python3
"""在电脑上本地运行 bPuppy 的网页调试界面 —— 不需要把电脑连到机器狗的热点。

用法::

    python3 tools/web_ui_mock.py                 # 拉一次真实页面后本地起服务
    python3 tools/web_ui_mock.py --port 9000     # 换端口
    python3 tools/web_ui_mock.py --offline       # 只用本地缓存的页面

它做三件事：

1. 从 ``http://192.168.4.1/`` 拉一次**真实页面**并缓存到 ``page_cache.html``
   （这一步需要电脑能访问到狗子；之后就不再需要）。
2. 在 ``127.0.0.1`` 起一个 HTTP 服务，吐出同一个页面，并用一个**有状态的
   mock** 接管 ``/cmd`` 接口（校准值、运动参数都会真的变，和板子行为一致）。
3. 浏览器打开 ``http://127.0.0.1:<port>/`` 就能点「设置」里的各个面板。

调试 UI 时用它：改 ``camera_on.py`` 的 HTML/JS → 重新拉页面 → 刷新浏览器，
全程不碰机器狗，也就不会因为舵机动作把板子搞重启。

注意：页面带 ``Cache-Control: no-store``，但**浏览器仍可能缓存旧页面**。
换了页面内容后如果看到的行为不对，先换端口或加个查询参数（``/?v=2``）再开。
"""

import argparse
import http.server
import json
import re
import sys
import urllib.parse
from pathlib import Path

CACHE = Path(__file__).with_name("page_cache.html")
DOG = "192.168.4.1"

# 8 路舵机零偏（度），与板子 NVS 里的值对应
CAL = [90.0] * 8
PARAMS = {"SPEED": 2.5, "STRIDE": 70.0, "HEIGHT": 70.0, "LIFT": 30.0,
          "OMEGA": 2.0, "TURN": 0.0, "DIR": 1}
BALANCE_ON = False
HEADING_ON = False


def fetch_dog_page(timeout=8):
    """从机器狗拉一次真实页面（含 UI 补丁注入后的结果）。"""
    import socket
    s = socket.socket()
    s.settimeout(timeout)
    s.connect((DOG, 80))
    s.send(b"GET / HTTP/1.0\r\nHost: x\r\n\r\n")
    buf = b""
    while True:
        try:
            d = s.recv(2048)
        except Exception:
            break
        if not d:
            break
        buf += d
    s.close()
    body = buf.split(b"\r\n\r\n", 1)[1]
    if len(body) < 1000:
        raise RuntimeError("页面只有 %d 字节，可能没连上机器狗" % len(body))
    return body


class Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"

    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/":
            self._send(PAGE, "text/html; charset=utf-8",
                       ["Cache-Control: no-store, no-cache, must-revalidate",
                        "Pragma: no-cache"])
            return
        if parsed.path == "/cmd":
            q = parsed.query
            if "diag=" in q:
                print("[BP-DIAG]", q[:200])
            elif "get_params=1" not in q:
                print("[CMD]", q[:90])
            self._send(self.cmd(q).encode("utf-8"), "text/plain; charset=utf-8",
                       ["Cache-Control: no-store"])
            return
        self.send_error(404)

    def _send(self, data, ctype, extra=()):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        for h in extra:
            self.send_header(*h.split(": ", 1))
        self.end_headers()
        self.wfile.write(data)

    def cmd(self, q):
        global CAL, BALANCE_ON, HEADING_ON
        if "diag=" in q:
            return "OK"
        if "get_cal=1" in q:
            return ",".join("%.1f" % v for v in CAL)
        if "cal=1" in q:
            p = dict(kv.split("=") for kv in q.split("&") if "=" in kv)
            CAL[int(p.get("ch", 0))] = float(p.get("deg", 90.0))
            return "OK:cal"
        if "all90=1" in q:
            CAL = [90.0] * 8
            return "OK:all90"
        if "sys_info=1" in q:
            return json.dumps({"v": 7.42, "pct": 86, "ram": 123456})
        if "cal_imu=1" in q:
            return "ERR:未检测到 IMU，请检查接线"
        if "get_params=1" in q:
            return "&".join("%s=%.1f" % (k, v) for k, v in PARAMS.items())
        if "set=1" in q and "speed=" in q:
            m = re.search(r"speed=([0-9.]+)", q)
            if m:
                PARAMS["SPEED"] = float(m.group(1))
            return "OK:set"
        for key in ("stride", "height", "lift"):
            if key + "=" in q:
                m = re.search(key + r"=([0-9.]+)", q)
                if m:
                    PARAMS[key.upper()] = float(m.group(1))
                return "OK"
        if "balance=toggle" in q:
            BALANCE_ON = not BALANCE_ON
            return "自平衡已开启" if BALANCE_ON else "自平衡已关闭"
        if "heading=off" in q:
            HEADING_ON = False
            return "指南小狗已关闭"
        if "heading=on" in q:
            HEADING_ON = True
            return "指南小狗已开启"
        if "heading=state" in q:
            return "ON" if HEADING_ON else "OFF"
        for p in ("pose=stand", "gait=sit", "gait=stop", "wave=1"):
            if p in q:
                return "OK:" + p.split("=")[0]
        return "OK"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--offline", action="store_true", help="只用本地缓存的页面")
    args = ap.parse_args()

    global PAGE
    if args.offline:
        PAGE = CACHE.read_bytes()
        print("使用缓存页面 %s（%d 字节）" % (CACHE, len(PAGE)))
    elif CACHE.exists():
        try:
            PAGE = fetch_dog_page()
            CACHE.write_bytes(PAGE)
            print("已从机器狗拉取最新页面（%d 字节）并更新缓存" % len(PAGE))
        except Exception as e:
            PAGE = CACHE.read_bytes()
            print("拉取失败（%s），改用缓存页面（%d 字节）" % (e, len(PAGE)))
    else:
        PAGE = fetch_dog_page()
        CACHE.write_bytes(PAGE)
        print("已从机器狗拉取页面（%d 字节）并缓存到 %s" % (len(PAGE), CACHE))

    print("本地调试服务: http://127.0.0.1:%d/  （Ctrl-C 结束）" % args.port)
    http.server.ThreadingHTTPServer(("127.0.0.1", args.port), Handler).serve_forever()


if __name__ == "__main__":
    sys.exit(main())
