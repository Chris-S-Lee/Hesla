# hid_axis_test.py
import hid, time

h = hid.device()
h.open(0x0e8f, 0x0003)
h.set_nonblocking(1)
print("컨트롤러 조작해보세요 (Ctrl+C로 종료)")

while True:
    data = h.read(64)
    if data:
        print(data)
    time.sleep(0.05)