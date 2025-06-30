import socket
import base64
import serial

COM = 'COM11'                   # RTK模块的串口号
BPS = 115200                    # RTK模块的串口波特率
NtripIP = '114.111.30.88'     # Ntrip的IP地址
NtripPort = 8003                # Ntrip的端口号
NtripPoint = 'AUTO'             # Ntrip的挂载点
NtripUser =  'qx4724'          # Ntrip的用户名
NtripPwd  = 'ac580963'              # Ntrip的密码


# ------------------------------------------------------------------
RTK = serial.Serial(COM, BPS, timeout=0.01)
print("等待RTK模块定位...")
while True:
    data = RTK.readline()
    if len(data):
        strNMEA = data.decode("ascii")
        seg = strNMEA.split(',')
        if seg[0] == "$GNGGA":
            if len(seg[6]) and seg[6]!='0':
                strGNGGA = strNMEA + "\r\n\r\n"
                print(strGNGGA)
                break

ntrip = socket.socket()
ntrip.connect((NtripIP,NtripPort))

user_pwd = base64.b64encode(bytes(NtripUser+':'+NtripPwd, 'utf-8')).decode("utf-8")
httpHead = "GET /"+NtripPoint+" HTTP/1.0\r\nUser-Agent: NTRIP GNSSInternetRadio/1.4.10\r\nAccept: */*\r\nConnection: close\r\nAuthorization: Basic "+user_pwd+"\r\n\r\n"
ntrip.send(httpHead.encode())
data = ntrip.recv(1024)
print(data)
ntrip.send(strGNGGA.encode())

while True:
    data = RTK.read(102400)
    if len(data):
        print(data.decode("ascii"))
    data = ntrip.recv(102400)
    RTK.write(data)
exit()
