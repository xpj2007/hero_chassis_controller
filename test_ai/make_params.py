#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate a copy of controllers.yaml with parameter overrides (test only).

usage: make_params.py OUT.yaml [section/key=value ...]
  keys are looked up at the top level, e.g.
  make_params.py /tmp/p.yaml mode=0 zero_speed_threshold=1.0
  (the controller section is applied automatically)
"""

import os
import sys

import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.normpath(os.path.join(_HERE, "..", "config", "controllers.yaml"))
_ROOT = "controller/hero_chassis_controller"


def set_path(tree, path, value):
    keys = path.split("/")
    node = tree
    for key in keys[:-1]:
        node = node.setdefault(key, {})
    node[keys[-1]] = value


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    out = sys.argv[1]
    with open(_SRC) as f:
        tree = yaml.safe_load(f)
    for item in sys.argv[2:]:
        if "=" not in item:
            print("bad override", item)
            return 2
        key, value = item.split("=", 1)
        set_path(tree, _ROOT + "/" + key, yaml.safe_load(value))
    with open(out, "w") as f:
        yaml.safe_dump(tree, f, default_flow_style=False, sort_keys=False)
    print("wrote", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
