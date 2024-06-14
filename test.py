from vamp_iface import VampInterface
from urx_iface import UrxIface

vamp_iface = VampInterface()

urx_iface = UrxIface()

start = urx_iface.get_config()
print(f"start config: {start}")

random_pos = vamp_iface.generate_random_config([])
plan = vamp_iface.plan(start, random_pos, [])
current = start
urx_iface.set_speed(1, 2)
for config in plan:
    print("====================================")
    print(f"current config: {current}")
    print(f"target config: {config}")
    urx_iface.move_to_config(config)
    current = config
