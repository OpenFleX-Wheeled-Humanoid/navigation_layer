"""Build a Livox runtime JSON from the per-robot OpenFlex YAML config."""

import ipaddress
import json
import os
import tempfile
from pathlib import Path

import yaml


def _ipv4(value, field_name):
    try:
        return str(ipaddress.IPv4Address(value))
    except ipaddress.AddressValueError as exc:
        raise RuntimeError(f'{field_name} 不是有效的 IPv4 地址: {value}') from exc


def merge_lidar_config(base_config_path, user_config_path=None):
    """Return a Livox JSON path updated from ~/.openflex/lidar_config.yaml."""
    base_path = Path(base_config_path).expanduser().resolve()
    if not base_path.is_file():
        raise RuntimeError(f'Livox 基础配置不存在: {base_path}')

    user_path = Path(
        user_config_path or '~/.openflex/lidar_config.yaml'
    ).expanduser()
    if not user_path.is_file():
        return str(base_path)

    with base_path.open('r', encoding='utf-8') as stream:
        config = json.load(stream)
    with user_path.open('r', encoding='utf-8') as stream:
        user_config = yaml.safe_load(stream) or {}

    try:
        host_ip = _ipv4(user_config['host_network']['ip'], 'host_network.ip')
        lidar_ip = _ipv4(
            user_config['lidars']['mid360']['device_ip'],
            'lidars.mid360.device_ip',
        )
    except (KeyError, TypeError) as exc:
        raise RuntimeError(
            f'雷达用户配置字段不完整: {user_path}; '
            '需要 host_network.ip 和 lidars.mid360.device_ip'
        ) from exc

    mid360 = config.get('Mid360s') or config.get('MID360')
    if not isinstance(mid360, dict):
        raise RuntimeError(f'Livox 基础配置缺少 Mid360s: {base_path}')

    host_info = mid360.get('host_net_info')
    lidar_configs = config.get('lidar_configs')
    if not isinstance(host_info, dict) or not lidar_configs:
        raise RuntimeError(f'Livox 基础配置网络字段不完整: {base_path}')

    for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
        if key in host_info:
            host_info[key] = host_ip
    lidar_configs[0]['ip'] = lidar_ip

    runtime_dir = Path(tempfile.gettempdir()) / f'openflex_livox_{os.getuid()}'
    runtime_dir.mkdir(parents=True, exist_ok=True)
    runtime_path = runtime_dir / 'MID360_config.json'
    temporary_path = runtime_path.with_suffix('.json.tmp')
    with temporary_path.open('w', encoding='utf-8') as stream:
        json.dump(config, stream, indent=2)
        stream.write('\n')
    os.replace(temporary_path, runtime_path)
    return str(runtime_path)
