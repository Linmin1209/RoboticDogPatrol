# hikvision_isapi.py
# =============================================================
"""
Hikvision ISAPI 简易封装：
- 实时视频流 (HTTP-MJPEG / RTSP)
- 红外视频流
- PTZ 云台控制
依赖: requests, opencv-python
"""

import re
import cv2
import time
import queue
import threading
import requests
from requests.auth import HTTPDigestAuth


class HikvisionISAPI:
    def __init__(
        self,
        ip: str,
        username: str,
        password: str,
        http_port: int = 80,
        rtsp_port: int = 554,
        timeout: int = 10,
    ):
        """
        ip           : 相机 IP
        username/psw : 海康相机登录用户名/密码
        http_port    : ISAPI HTTP 端口 (默认 80，如改过需对应)
        rtsp_port    : RTSP 端口 (默认 554)
        """
        self.ip = ip
        self.username = username
        self.password = password
        self.auth = HTTPDigestAuth(username, password)
        self.base_http = f"http://{ip}:{http_port}/ISAPI"
        self.base_rtsp = f"rtsp://{username}:{password}@{ip}:{rtsp_port}"

        self._timeout = timeout

    # ------------------------------------------------------------------
    # 1. 获取流 URL（HTTP MJPEG 或 RTSP）
    # ------------------------------------------------------------------
    def _channel_str(self, channel: int | str) -> str:
        """确保通道格式正确：int 101 -> '101'，str '101' -> '101'"""
        return str(channel)

    def mjpeg_http_url(self, channel: int | str = 102) -> str:
        """
        返回 HTTP-MJPEG 预览 URL（需把该通道编码设置为 MJPEG）
        通常 101=主码流(H.264/H.265)，102=子码流(MJPEG)，
        IR/热成像常在 201/401，具体看设备。
        """
        ch = self._channel_str(channel)
        return f"{self.base_http}/Streaming/channels/{ch}/httpPreview"

    def rtsp_url(self, channel: int | str = 101) -> str:
        """返回 RTSP URL，可用在 VLC、CV2 等"""
        ch = self._channel_str(channel)
        return f"{self.base_rtsp}/Streaming/Channels/{ch}"

    # ------------------------------------------------------------------
    # 2. MJPEG 流读取（纯 Python，无需 FFmpeg）
    # ------------------------------------------------------------------
    def iter_mjpeg_frames(self, channel: int | str = 102, chunk: int = 1024):
        """
        逐帧生成器：yield (bgr_frame, timestamp)
        注意：需提前将该“通道”设置为 “MJPEG” 编码，否则返回 H264 数据无法解析。
        """
        url = self.mjpeg_http_url(channel)
        with requests.get(url, auth=self.auth, stream=True, timeout=self._timeout) as r:
            r.raise_for_status()
            # 从 header 里找边界串
            m = re.search(r'boundary=(.*)', r.headers.get("Content-Type", ""), re.I)
            boundary = (m.group(1) if m else "--myboundary").encode()
            buff = b""
            for chunk_bytes in r.iter_content(chunk_size=chunk):
                buff += chunk_bytes
                while True:
                    start = buff.find(boundary)
                    if start == -1:
                        break
                    end = buff.find(boundary, start + len(boundary))
                    if end == -1:
                        break
                    part = buff[start + len(boundary):end]
                    buff = buff[end:]
                    # 每个 part = header + \r\n\r\n + jpeg
                    head_end = part.find(b"\r\n\r\n")
                    if head_end != -1:
                        jpeg = part[head_end + 4:]
                        frame = cv2.imdecode(
                            np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR
                        )
                        if frame is not None:
                            yield frame, time.time()

    # ------------------------------------------------------------------
    # 3. PTZ 控制
    # ------------------------------------------------------------------
    def _put_xml(self, url: str, xml: str):
        hdr = {"Content-Type": "application/xml"}
        resp = requests.put(url, data=xml.encode(), auth=self.auth, headers=hdr, timeout=self._timeout)
        resp.raise_for_status()
        return resp

    # ---- 连续 (pan/tilt/zoom) ------------------------------------------------
    def ptz_move(
        self,
        pan: int = 0,
        tilt: int = 0,
        zoom: int = 0,
        channel: int = 1,
        speed: int = 40,
        stop: bool = False,
    ):
        """
        连续 PTZ：pan/tilt/zoom 取值范围 -100~100，speed 1~100
        stop=True 时会发送 /stop 指令。
        """
        if stop:
            url = f"{self.base_http}/PTZCtrl/channels/{channel}/continuous"
            xml = f"""
        <PTZData xmlns="http://www.hikvision.com/ver20/XMLSchema">
            <pan>0</pan>
            <tilt>0</tilt>
            <zoom>0</zoom>
            <speed>0</speed>
        </PTZData>
        """
            return self._put_xml(url, xml)

        url = f"{self.base_http}/PTZCtrl/channels/{channel}/continuous"
        xml = f"""
        <PTZData xmlns="http://www.hikvision.com/ver20/XMLSchema">
            <pan>{pan}</pan>
            <tilt>{tilt}</tilt>
            <zoom>{zoom}</zoom>
            <speed>{speed}</speed>
        </PTZData>
        """
        return self._put_xml(url, xml)

    # ---- 预置位 --------------------------------------------------------------
    def goto_preset(self, preset_no: int, channel: int = 1):
        url = f"{self.base_http}/PTZCtrl/channels/{channel}/presets/{preset_no}/goto"
        return self._put_xml(url, "<PTZData><mode>normal</mode></PTZData>")

    def set_preset(self, preset_no: int, channel: int = 1):
        url = f"{self.base_http}/PTZCtrl/channels/{channel}/presets/{preset_no}"
        xml = f"<PTZPreset><id>{preset_no}</id></PTZPreset>"
        return self._put_xml(url, xml)

    # ------------------------------------------------------------------
    # 4. 简易演示（opencv 窗口 + 键盘控制）
    # ------------------------------------------------------------------
    def demo(self, ir_channel: int | str = 102):
        """
        方向键控制云台，'z'/'x' 控制 zoom in/out，'s' 停止，'q' 退出
        """
        print("[INFO] 键盘←↑→↓控制 PTZ, z=Zoom+, x=Zoom-, s=Stop, q=Quit")
        cap = cv2.VideoCapture(self.rtsp_url(ir_channel))
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            cv2.imshow("Hikvision Live", frame)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            elif k in (81, 82, 83, 84):  # ←↑→↓
                pan, tilt = {81: (-30, 0), 82: (0, 30), 83: (30, 0), 84: (0, -30)}[k]
                self.ptz_move(pan=pan, tilt=tilt)
            elif k == ord("z"):
                self.ptz_move(zoom=20)
            elif k == ord("x"):
                self.ptz_move(zoom=-20)
            elif k == ord("s"):
                self.ptz_move(stop=True)
        cap.release()
        cv2.destroyAllWindows()

    def preview_rgb_ir(self, 
                    color_channel: int | str = 101,
                    ir_channel: int | str = 401,   # 有的机型是 201 / 103
                    resize_long_edge: int | None = 800):
        #打开 RTSP 流
        cap_color = cv2.VideoCapture(self.rtsp_url(color_channel))
        cap_ir    = cv2.VideoCapture(self.rtsp_url(ir_channel))

        if not cap_color.isOpened():
            raise RuntimeError(f"彩色视频流 {color_channel} 打不开")
        if not cap_ir.isOpened():
            raise RuntimeError(f"红外流 {ir_channel} 打不开")

        print("[INFO] 按 q 退出预览")
        while True:
            ret_c, frame_c = cap_color.read()
            ret_i, frame_i = cap_ir.read()
            if not (ret_c and ret_i):
                print("⚠️ 读取失败，3 秒后重试…")
                time.sleep(3)
                continue

            # 红外通常是灰度 → 伪彩或三通道显示
            if len(frame_i.shape) == 2 or frame_i.shape[2] == 1:
                frame_i = cv2.applyColorMap(frame_i, cv2.COLORMAP_JET)

            # 可选统一缩放
            if resize_long_edge:
                def _resize(img):
                    h, w = img.shape[:2]
                    scale = resize_long_edge / max(h, w)
                    if scale < 1.0:
                        img = cv2.resize(img, (int(w*scale), int(h*scale)))
                    return img
                frame_c = _resize(frame_c)
                frame_i = _resize(frame_i)

            # 分别显示
            cv2.imshow("Color Stream", frame_c)
            cv2.imshow("Infrared Stream", frame_i)

            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break

        cap_color.release()
        cap_ir.release()
        cv2.destroyAllWindows()


# ----------------------------- 快速测试 -------------------------------------
if __name__ == "__main__":
    cam = HikvisionISAPI("192.168.1.64", "admin", "Robotdog1")
    # 示例：云台向右移动 2 秒并停止
    # cam.ptz_move(pan=30)
    # time.sleep(2)
    # cam.ptz_move(stop=True)
    cam.preview_rgb_ir()
