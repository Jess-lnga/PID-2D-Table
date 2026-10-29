"""Build an offline UI preview without connecting to an ESP32 or moving hardware."""
import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('output', type=Path)
parser.add_argument('--stage', type=int, default=1, choices=range(5))
args = parser.parse_args()
src = Path(__file__).resolve().parents[2] / 'src'
wifi = (src/'wifi_interface.cpp').read_text(encoding='utf-8')
cal = (src/'servo_calibration_web.cpp').read_text(encoding='utf-8')
page = wifi.split('R"rawliteral(')[1].split(')rawliteral"')[0]
page = page.replace('<!--SERVO_CAL_MENU-->', cal.split('R"calhtml(')[1].split(')calhtml"')[0])
page = page.replace('/*SERVO_CAL_SCRIPT*/', cal.split('R"caljs(')[1].split(')caljs"')[0])
state = dict(menuActive=True, stage=args.stage, session=42, revision=5, servo=1,
             point=2, targetDeg=45, message='', minTicks=1639, maxTicks=8191,
             draft=dict(valid=0, joint=0, ticks=[3300, 4000, 0, 0]),
             records=[dict(valid=0, joint=0, ticks=[0]*4) for _ in range(3)],
             actualTicks=[4900, 4100, 4300])
if args.stage == 3:
    state['draft'] = dict(valid=1, joint=1, ticks=[3300, 4000, 5000, 6000])
    state['targetDeg'] = -30
    state['actualTicks'][0] = 3300
page = page.replace('window.onload = updateState;', 'window.onload = () => { rememberCalToken(42); renderServoCalibration('+json.dumps(state)+'); };')
args.output.write_text(page, encoding='utf-8')
print(args.output.as_uri())
