#!/usr/bin/env python3
from elite_cs_sdk import DashboardClientInterface
import time

if __name__ == "__main__":
    ip = '192.168.1.200'  # 这里改成你机器人的 IP，通常是和 ros2 控制用的那个一致

    dash = DashboardClientInterface()
    ok = dash.connect(ip, 29999)
    print("connect:", ok)
    if not ok:
        exit(1)

    print("calling powerOn() ...")
    t0 = time.time()
    try:
        ret = dash.powerOn()
        t1 = time.time()
        print("powerOn() returned:", ret, "elapsed:", t1 - t0, "seconds")
    except Exception as e:
        t1 = time.time()
        print("powerOn() raised exception:", e, "elapsed:", t1 - t0, "seconds")

    dash.disconnect()
