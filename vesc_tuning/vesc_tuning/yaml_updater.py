"""Targeted, comment-preserving updater for the F1TENTH vesc.yaml.

vesc.yaml stores its parameters under a `/**:` wildcard namespace, e.g.::

    /**:
      ros__parameters:
        speed_to_erpm_gain: 4614.0
        steering_angle_to_servo_gain: -1.2135

PyYAML is used to *parse and validate* that the requested key exists (and under
which namespace), but the actual write is a targeted single-line regex edit so
that comments, ordering and untouched values are preserved (a full yaml.dump
would strip every comment).
"""

import os
import re
import tempfile

import yaml


def _format_value(value: float) -> str:
    """Render a float compactly but unambiguously (always with a decimal point)."""
    text = repr(float(value))
    # repr keeps full precision; ensure it always looks like a float
    if 'e' not in text and 'E' not in text and '.' not in text:
        text += '.0'
    return text


def _key_present(data, param_name: str) -> bool:
    """True if param_name exists under '/**'/ros__parameters or at the document root."""
    if not isinstance(data, dict):
        return False
    wildcard = data.get('/**')
    if isinstance(wildcard, dict):
        ros_params = wildcard.get('ros__parameters')
        if isinstance(ros_params, dict) and param_name in ros_params:
            return True
    # fallback: key directly at root (defensive)
    return param_name in data


def read_vesc_param(yaml_path: str, param_name: str):
    """Return the current value of ``param_name`` from vesc.yaml, or None.

    Looks under the ``/**`` wildcard namespace (or document root, as a fallback).
    Returns None if the file is missing/unparseable or the key is absent, so the
    caller can fall back to its own default. Never raises.
    """
    if not yaml_path or not os.path.isfile(yaml_path):
        return None
    try:
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f.read())
    except (OSError, yaml.YAMLError) as exc:
        print(f'[yaml_updater] WARN: cannot read "{yaml_path}": {exc}')
        return None
    if not isinstance(data, dict):
        return None
    wildcard = data.get('/**')
    if isinstance(wildcard, dict):
        ros_params = wildcard.get('ros__parameters')
        if isinstance(ros_params, dict) and param_name in ros_params:
            return ros_params[param_name]
    if param_name in data:
        return data[param_name]
    return None


def update_vesc_yaml(yaml_path: str, param_name: str, new_value: float) -> bool:
    """Update a single scalar parameter in vesc.yaml in place.

    Reads the YAML, confirms ``param_name`` exists under the ``/**`` wildcard
    namespace (or at root level), updates its value preserving all other content
    and comments, and writes the file back.

    Returns True on success, False on failure. Prints a clear error message if
    the file cannot be parsed or written.
    """
    if not yaml_path:
        print('[yaml_updater] ERROR: no yaml_path provided.')
        return False
    if not os.path.isfile(yaml_path):
        print(f'[yaml_updater] ERROR: file not found: {yaml_path}')
        return False

    # --- read + validate with PyYAML --------------------------------------
    try:
        with open(yaml_path, 'r') as f:
            text = f.read()
        data = yaml.safe_load(text)
    except (OSError, yaml.YAMLError) as exc:
        print(f'[yaml_updater] ERROR: cannot parse YAML "{yaml_path}": {exc}')
        return False

    if not _key_present(data, param_name):
        print(f'[yaml_updater] ERROR: parameter "{param_name}" not found under '
              f'"/**: ros__parameters" (or root) in {yaml_path}. File left unchanged.')
        return False

    # --- targeted line edit (preserves comments / layout) -----------------
    # Match:  <indent><key><spaces>:<spaces><value><optional trailing comment/space>
    pattern = re.compile(
        r'^(?P<head>[ \t]*' + re.escape(param_name) + r'[ \t]*:[ \t]*)'
        r'(?P<value>[^\s#]+)'
        r'(?P<tail>[ \t]*(?:#.*)?)$',
        re.MULTILINE,
    )

    new_scalar = _format_value(new_value)

    def _repl(match: 're.Match') -> str:
        return f"{match.group('head')}{new_scalar}{match.group('tail')}"

    new_text, n_subs = pattern.subn(_repl, text, count=1)
    if n_subs == 0:
        print(f'[yaml_updater] ERROR: could not locate a scalar line for '
              f'"{param_name}" to edit in {yaml_path}. File left unchanged.')
        return False

    try:
        with open(yaml_path, 'w') as f:
            f.write(new_text)
    except OSError as exc:
        print(f'[yaml_updater] ERROR: cannot write "{yaml_path}": {exc}')
        return False

    print(f'[yaml_updater] Set {param_name} = {new_scalar} in {yaml_path}')
    return True


def atomic_write_calibration_yaml(yaml_path: str, values: dict) -> None:
    """(Over)write yaml_path as a `/**: ros__parameters:` document holding
    exactly `values`, atomically: written to a temp file in the same
    directory first, then os.replace()'d over the target, so a crash or
    Ctrl-C mid-write can never leave a real node (which may read this file
    at any moment, e.g. on its next restart) a partially-written file.

    Unlike update_vesc_yaml above (a targeted single-line edit, chosen
    specifically to preserve comments/ordering in a large, hand-maintained
    shared file like vesc.yaml), this regenerates the whole file from
    scratch every call. That's the right tradeoff here because the caller
    (steering_calibration_node.py) treats its target file as fully
    machine-managed and holding nothing but these keys -- there is no
    hand-written content to lose.

    Raises OSError on failure (temp-file write or the final replace) --
    callers decide how to surface that; nothing here is swallowed silently,
    since a failed write means the file a real node depends on was NOT
    updated and the caller's in-memory value and on-disk value have now
    diverged.
    """
    lines = [
        "# Auto-generated and live-updated by vesc_tuning's "
        'steering_calibration_node.py --',
        '# hand edits will be overwritten on its next write. See that node\'s '
        'module docstring.',
        '/**:',
        '  ros__parameters:',
    ]
    for key, value in values.items():
        lines.append(f'    {key}: {_format_value(value)}')
    text = '\n'.join(lines) + '\n'

    directory = os.path.dirname(os.path.abspath(yaml_path)) or '.'
    fd, tmp_path = tempfile.mkstemp(
        prefix='.steering_calibration_', suffix='.yaml.tmp', dir=directory)
    try:
        with os.fdopen(fd, 'w') as f:
            f.write(text)
        os.replace(tmp_path, yaml_path)
    except OSError:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
        raise
