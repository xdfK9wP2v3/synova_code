import datetime
import numpy as np
import matplotlib.pyplot as plt

TICKS_PER_SECOND = 10000

times, observe, ema = [], [], []

with open('log.log', 'r') as f:
    while line := f.readline():
        line = line.strip()
        if len(line) <= 4 or line[0] != '$' or line[-3] != '*':
            continue
        payload = line[1:-3]
        checksum: int = 0
        for b in payload.encode():
            checksum = (checksum ^ b) & 0xff
        if int(line[-2:], 16) != checksum:
            continue

        msg_type, msg_idx, sys_state, _, _, timestamp, second, tick, sub_tick, clock, *fields = payload.split(',')
        msg_idx, sys_state, second, tick, (sub_tick, tick_frac), clock = int(msg_idx), int(sys_state, 16), int(second), int(tick), sub_tick.split('/'), int(clock)
        sub_tick, tick_frac = int(sub_tick), int(tick_frac)
        sub_second = (tick + sub_tick / tick_frac) / TICKS_PER_SECOND
        date_time = datetime.datetime.utcfromtimestamp(int(timestamp) + sub_second) if len(timestamp) > 0 else None
        curr_time = second + sub_second

        if msg_type == 'PULSE':
            prev_clk_per_pulse, curr_clk_per_pulse = fields
            curr_clk_per_pulse, sub_clk = curr_clk_per_pulse.split('+')
            sub_clk = sub_clk.split('/')
            curr_clk_per_pulse = int(curr_clk_per_pulse) + int(sub_clk[0]) / int(sub_clk[1])

            times.append(curr_time)
            observe.append(int(prev_clk_per_pulse))
            ema.append(int(prev_clk_per_pulse))

times = np.array(times)
observe = np.array(observe)

plt.figure(figsize=(12, 6))
s = slice(None)
plt.plot(times[s], observe[s], '.')
plt.plot(times[s], ema[s], '.')
plt.show()
plt.plot(times[s][:-1], np.diff(observe[s]), '.')
plt.show()
