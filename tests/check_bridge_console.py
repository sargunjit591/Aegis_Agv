#!/usr/bin/env python3
"""Static checks on the motor bridge and the console. No imports of either
file, no ROS, no hardware — pure AST.

  1. nothing is defined twice (functions, methods, constants, ROS parameters,
     dict keys, Flask routes) — CLAUDE.md rule 0.3
  2. both files carry the same pin map and shared constants, and they match
     the verified hardware table in CLAUDE.md §4 — rule 0.5
  3. find_header_chip() is the same code in both files

Run:  python tests/check_bridge_console.py
"""
import ast
import collections
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
BRIDGE = ROOT / 'ubuntu_ros2' / 'agv_motor_bridge.py'
CONSOLE = ROOT / 'rpi_os_console' / 'agv_console_angle.py'

# console constant -> bridge parameter, and the verified value (CLAUDE.md §4)
SHARED = [
    ('STEER_RPWM', 'steer_rpwm_pin', 19),
    ('STEER_LPWM', 'steer_lpwm_pin', 16),
    ('STEER_ACTIVE_LOW', 'steer_active_low', False),
    ('GRIP_RPWM', 'grip_rpwm_pin', 13),
    ('GRIP_LPWM', 'grip_lpwm_pin', 12),
    ('GRIP_ACTIVE_LOW', 'grip_active_low', True),
    ('RELAY_DIR_PIN', 'relay_dir_pin', 5),
    ('BRAKE_PIN', 'brake_pin', 6),
    ('BRAKE_ENGAGE_LEVEL', 'brake_engage_level', 0),
    ('BRAKE_ENABLE', 'brake_enable', True),
    ('BRAKE_RELEASE_LEAD_S', 'brake_release_lead_s', 0.35),
    ('BRAKE_MAX_ENGAGE_S', 'brake_max_engage_s', 2.0),
    ('DAC_CS_PIN', 'dac_cs_pin', 23),
    ('HAT_VREF', 'dac_vref', 5.0),
    ('THROTTLE_IDLE_V', 'throttle_idle_v', 0.80),
    ('THROTTLE_MAX_V', 'throttle_max_v', 4.20),
    ('AS5600_BUS', 'as5600_bus', 1),
    ('AS5600_CENTER_RAW', 'as5600_center_raw', 3791),
    ('AS5600_CCW_POSITIVE', 'as5600_ccw_positive', False),
    ('STEER_MAX_DEG', 'steer_max_deg', 45.0),
    ('STEER_MAX_RUN_S', 'steer_max_run_s', 4.0),
    ('STEER_STOP_LEAD_S', 'steer_stop_lead_s', 0.12),
    ('STEER_RATE_ALPHA', 'steer_rate_alpha', 0.35),
    ('GPIOCHIP', 'gpiochip', -1),
]

failures = []


def check(ok, msg):
    print(('  ok   ' if ok else '  FAIL ') + msg)
    if not ok:
        failures.append(msg)


def dupes(names):
    return sorted(n for n, c in collections.Counter(names).items() if c > 1)


def analyse(path):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    info = {'consts': {}, 'funcs': {}, 'params': None}
    top_funcs, top_assigns, routes, declared, used = [], [], [], [], []

    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.ClassDef)):
            top_funcs.append(node.name)
            info['funcs'][node.name] = node
        elif isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name):
                    top_assigns.append(t.id)
                    try:
                        info['consts'][t.id] = ast.literal_eval(node.value)
                    except Exception:
                        pass

    check(not dupes(top_funcs), f'{path.name}: no duplicate top-level def/class '
                                f'({len(top_funcs)} defined) {dupes(top_funcs)}')
    check(not dupes(top_assigns), f'{path.name}: no duplicate top-level assignment '
                                  f'({len(top_assigns)} names) {dupes(top_assigns)}')

    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef):
            methods = [n.name for n in node.body if isinstance(n, ast.FunctionDef)]
            check(not dupes(methods), f'{path.name}: class {node.name} has no duplicate '
                                      f'method ({len(methods)} methods) {dupes(methods)}')
        elif isinstance(node, ast.Dict):
            keys = [k.value for k in node.keys if isinstance(k, ast.Constant)]
            if dupes(keys):
                check(False, f'{path.name}: dict literal at line {node.lineno} '
                             f'repeats keys {dupes(keys)}')
        elif isinstance(node, ast.FunctionDef):
            for d in node.decorator_list:
                if (isinstance(d, ast.Call) and isinstance(d.func, ast.Attribute)
                        and d.func.attr == 'route'):
                    routes.append(d.args[0].value)
        elif isinstance(node, ast.Call):
            f = node.func
            name = f.attr if isinstance(f, ast.Attribute) else getattr(f, 'id', '')
            if name == 'declare_parameter':
                declared.append(node)
            elif name == 'g' and node.args and isinstance(node.args[0], ast.Constant):
                used.append(node.args[0].value)

    if routes:
        check(not dupes(routes), f'{path.name}: no duplicate Flask route '
                                 f'({len(routes)} routes) {dupes(routes)}')
    info['declared'], info['used'] = declared, used
    return info


print('== duplicates ==')
bridge = analyse(BRIDGE)
console = analyse(CONSOLE)

print('== bridge ROS parameters ==')
params = bridge['consts'].get('PARAMS', {})
check(len(bridge['declared']) == 1,
      f'declare_parameter is called from exactly one place '
      f'(found {len(bridge["declared"])}): the loop over PARAMS')
check(len(params) > 0, f'PARAMS table has {len(params)} parameters, each key unique')
check(not dupes(bridge['used']), f'each parameter is read exactly once {dupes(bridge["used"])}')
check(set(bridge['used']) == set(params),
      f'parameters read == parameters declared '
      f'(undeclared: {sorted(set(bridge["used"]) - set(params))}, '
      f'unused: {sorted(set(params) - set(bridge["used"]))})')

print('== pin map and shared constants: console == bridge == CLAUDE.md ==')
for const, param, want in SHARED:
    c, b = console['consts'].get(const, '<missing>'), params.get(param, '<missing>')
    check(c == b == want and type(c) is type(b),
          f'{const:<22} {c!r:<7} | {param:<22} {b!r:<7} | verified {want!r}')
fwd_level = 1 if params.get('invert_direction') else 0
check(console['consts'].get('FORWARD_RELAY_LEVEL') == fwd_level == 0,
      f'direction relay: LOW = forward in both '
      f'(console FORWARD_RELAY_LEVEL={console["consts"].get("FORWARD_RELAY_LEVEL")}, '
      f'bridge invert_direction={params.get("invert_direction")})')
check(params.get('drive_volts_forward') == 1.50 and params.get('drive_volts_reverse') == 1.28,
      'bridge drive thresholds 1.50 V forward / 1.28 V reverse')

print('== gpiochip auto-detect ==')
fb, fc = bridge['funcs'].get('find_header_chip'), console['funcs'].get('find_header_chip')
check(fb is not None and fc is not None and ast.dump(fb) == ast.dump(fc),
      'find_header_chip() is identical code in both files')
for path in (BRIDGE, CONSOLE):
    src = path.read_text(encoding='utf-8')
    check('gpiochip_open(0)' not in src, f'{path.name}: no hardcoded gpiochip_open(0)')

print()
print(f'{len(failures)} FAILED' if failures else 'ALL CHECKS PASSED')
sys.exit(1 if failures else 0)
