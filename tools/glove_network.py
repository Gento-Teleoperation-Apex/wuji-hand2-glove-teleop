#!/usr/bin/env python3
"""Discover gloves and add temporary host IPv4 addresses on their wired interfaces.

Runs the SDK as the invoking user; only arping and ip address need sudo.
Never writes glove settings, replaces host addresses, or enables a hand.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import ipaddress as ip
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time


@dataclass(frozen=True)
class Glove:
    sn: str
    address: str
    host: ip.IPv4Address


@dataclass(frozen=True)
class Addition:
    interface: str
    cidr: str


@dataclass
class Plan:
    additions: list[Addition]
    interfaces: dict[str, str]  # glove SN -> interface proven by ARP


def run(command, *, privileged=False, check=True):
    if privileged and os.geteuid() != 0:
        command = ['sudo', '--', *command]
    result = subprocess.run(command, text=True, capture_output=True, timeout=15,
                            env={**os.environ, 'LC_ALL': 'C'})
    if check and result.returncode:
        raise RuntimeError(f"{' '.join(command)}: {result.stderr.strip() or result.stdout.strip()}")
    return result


class LinuxNetwork:
    def __init__(self):
        self.ip = shutil.which('ip')
        self.arping = shutil.which('arping')
        if not self.ip or not self.arping:
            raise RuntimeError('需要 iproute2 / iputils-arping：sudo apt install iproute2 iputils-arping')

    def addresses(self):
        # Ubuntu's `ip -j -4 address` omits link_type. Read link metadata
        # separately, including connected interfaces with no IPv4 address yet.
        links = json.loads(run([self.ip, '-j', 'link', 'show']).stdout)
        addresses = json.loads(run([self.ip, '-j', '-4', 'address', 'show']).stdout)
        by_index = {row['ifindex']: row.get('addr_info', []) for row in addresses}
        return [{**link, 'addr_info': by_index.get(link['ifindex'], [])} for link in links]

    def is_physical_wired(self, interface):
        path = Path('/sys/class/net') / interface
        return (path / 'device').exists() and not (path / 'wireless').exists()

    def wired(self, row):
        return (row.get('link_type') == 'ether' and 'UP' in row.get('flags', [])
                and 'LOWER_UP' in row.get('flags', []) and not row.get('master')
                and self.is_physical_wired(row['ifname']))

    def occupied(self, interface, host):
        # iputils DAD: 0 = no replies; 1 = an address owner replied; 2 = error.
        result = run([self.arping, '-D', '-I', interface, '-c', '2', '-w', '3', str(host)],
                     privileged=True, check=False)
        if result.returncode not in (0, 1) or result.stderr.strip():
            raise RuntimeError(f'ARP 检查失败 ({interface}, {host}): {result.stderr or result.stdout}')
        return result.returncode == 1

    def route(self, host):
        rows = json.loads(run([self.ip, '-j', '-4', 'route', 'get', str(host)]).stdout)
        if len(rows) != 1:
            raise RuntimeError(f'无法确定 {host} 的路由')
        return rows[0]

    def routes(self):
        return json.loads(run([self.ip, '-j', '-4', 'route', 'show', 'table', 'all']).stdout)

    def add(self, addition):
        run([self.ip, 'address', 'add', addition.cidr, 'dev', addition.interface], privileged=True)

    def remove(self, addition):
        run([self.ip, 'address', 'del', addition.cidr, 'dev', addition.interface], privileged=True)


def discover(manager, device_type, serials):
    found = {}
    for device in manager.scan():
        if device.device_type != device_type or (serials and device.sn not in serials):
            continue
        try:
            host, port = device.address.rsplit(':', 1)
            host = ip.IPv4Address(host)
            if not 1 <= int(port) <= 65535 or host.is_unspecified or host.is_multicast or host.is_loopback:
                raise ValueError('not a unicast IPv4 endpoint')
        except ValueError as exc:
            raise RuntimeError(f'手套 {device.sn} 的地址不支持自动配置：{device.address}') from exc
        glove = Glove(device.sn, device.address, host)
        if device.sn in found and found[device.sn] != glove:
            raise RuntimeError(f'同一个 SN 出现不同地址：{device.sn}')
        found[device.sn] = glove
    if not found:
        raise RuntimeError('没有发现所选手套。检查电源、网线与 SDK 扫描结果。')
    if serials and set(serials) - found.keys():
        raise RuntimeError(f'未发现指定手套：{sorted(set(serials) - found.keys())}')
    gloves = sorted(found.values(), key=lambda g: g.sn)
    if len({g.host for g in gloves}) != len(gloves):
        raise RuntimeError('不同手套报告相同 IP，需先解决设备 IP 冲突。')
    return gloves


def local_addresses(rows):
    return [(row['ifname'], ip.IPv4Interface(f"{a['local']}/{a['prefixlen']}"))
            for row in rows for a in row.get('addr_info', []) if a.get('family') == 'inet']


def build_plan(gloves, network, interface=None, prefix=24):
    rows = network.addresses()
    locals_ = local_addresses(rows)
    candidates = [r['ifname'] for r in rows if network.wired(r)]
    if interface:
        if interface not in candidates:
            raise RuntimeError(f'{interface} 不是已连接的独立物理有线接口；桥接/VLAN 需手动配置。')
        candidates = [interface]
    if not candidates:
        raise RuntimeError('没有已连接的物理有线接口。检查网线；不自动修改 Wi-Fi、桥接或虚拟网卡。')
    used = {a.ip for _, a in locals_} | {g.host for g in gloves}
    plan = Plan([], {})
    for glove in gloves:
        if any(glove.host == a.ip for _, a in locals_):
            raise RuntimeError(f'手套 {glove.sn} 地址 {glove.host} 与本机地址冲突。')
        print(f'探测 {glove.sn} ({glove.host}) 的有线接口…', flush=True)
        matches = [name for name in candidates if network.occupied(name, glove.host)]
        if len(matches) != 1:
            raise RuntimeError(f'{glove.host} 的 ARP 响应接口为 {matches}，无法唯一定位。'
                               '检查接线；多接口时使用 --interface eth0 指定。')
        name = matches[0]
        plan.interfaces[glove.sn] = name
        # Device subnet masks are not exposed by discovery. Use explicit/default
        # prefix for new addresses, reuse an existing matching address if possible.
        existing = [a for n, a in locals_ if n == name and glove.host in a.network]
        if existing:
            route = network.route(glove.host)
            if route.get('dev') != name or route.get('gateway'):
                raise RuntimeError(f'{glove.host} 已有同网段地址，但路由未直连 {name}；请检查策略路由。')
            continue
        subnet = ip.IPv4Network(f'{glove.host}/{prefix}', strict=False)
        if glove.host in (subnet.network_address, subnet.broadcast_address):
            raise RuntimeError(f'{glove.host} 不能使用 /{prefix}，请核实 --prefix-length。')
        if any(n != name and subnet.overlaps(a.network) for n, a in locals_):
            raise RuntimeError(f'{subnet} 与其他网卡地址重叠；不自动覆盖现有网络。')
        for route in network.routes():
            dst = route.get('dst', 'default')
            if dst == 'default' or dst == '0.0.0.0/0':
                continue
            route_net = ip.IPv4Network(dst, strict=False)
            if subnet.overlaps(route_net) and (route.get('dev') != name or route.get('gateway')):
                raise RuntimeError(f'{subnet} 与已有路由 {dst} 重叠，需手动检查。')
        planned = [ip.IPv4Interface(a.cidr) for a in plan.additions if a.interface == name]
        if any(glove.host in a.network for a in planned):
            continue
        if any(subnet.overlaps(ip.IPv4Interface(a.cidr).network) and a.interface != name
               for a in plan.additions):
            raise RuntimeError('不同接口上的手套网段重叠，需手动配置。')
        # Prefer .200 in a /24; bounded search, exclude device/local/network/broadcast IPs.
        offsets = list(range(200, min(255, subnet.num_addresses - 1)))
        offsets += list(range(2, min(200, subnet.num_addresses - 1)))
        for offset in offsets:
            candidate = subnet.network_address + offset
            if candidate not in used and not network.occupied(name, candidate):
                plan.additions.append(Addition(name, f'{candidate}/{prefix}'))
                used.add(candidate)
                break
        else:
            raise RuntimeError(f'{subnet} 的候选地址均不可用，请手动配置主机地址。')
    return plan


def verify_routes(gloves, plan, network):
    locals_ = local_addresses(network.addresses())
    for glove in gloves:
        name = plan.interfaces[glove.sn]
        route = network.route(glove.host)
        source = route.get('prefsrc', route.get('src'))
        valid = any(n == name and str(a.ip) == source and glove.host in a.network for n, a in locals_)
        if route.get('dev') != name or route.get('gateway') or not valid:
            raise RuntimeError(f'{glove.host} 路由/源地址仍不正确：{route}')


def verify_gloves(manager, gloves):
    from wuji_sdk import ConnectOptions
    options = ConnectOptions(timeout_ms=3000, retry_count=0, enable_bridge=False,
                             auto_time_sync_interval_ms=None)
    try:
        for glove in gloves:
            device = manager.connect(address=glove.address, device_name=f'network_{glove.sn}', options=options)
            if device.serial_number != glove.sn:
                raise RuntimeError(f'{glove.address} 返回了其他 SN；检查重复 IP。')
            side = str(device.hand_side().get())
            sub = device.hand_skeleton().subscribe()
            try:
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    frame = sub.recv()
                    if frame is not None and len(frame.joints) == 21:
                        points = [j.pose.position for j in frame.joints]
                        if all(len(p) == 3 and all(math.isfinite(v) for v in p) for p in points):
                            print(f'通过：{glove.sn} {side} {glove.address}，收到 21 点骨架')
                            break
                    time.sleep(.01)
                else:
                    raise RuntimeError(f'{glove.sn} 连接成功但没有有效骨架；检查手套状态/SDK。')
            finally:
                sub.close()
            manager.disconnect_all()
    finally:
        manager.disconnect_all()


def apply_plan(gloves, plan, network, verify):
    added = []
    try:
        for addition in plan.additions:
            # Recheck just before mutation, since planning may take several seconds.
            host = ip.IPv4Interface(addition.cidr).ip
            if network.occupied(addition.interface, host):
                raise RuntimeError(f'{host} 已被占用，请重新运行以选择其他地址。')
            network.add(addition)
            added.append(addition)
        verify_routes(gloves, plan, network)
        verify()
    except BaseException:
        for addition in reversed(added):
            try:
                network.remove(addition)
            except Exception as exc:
                print(f'回滚失败：{exc}\n手动恢复：sudo ip address del {addition.cidr} dev {addition.interface}',
                      file=sys.stderr)
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description='一/二代共用：发现手套并为机器人有线网卡补充同网段地址。')
    parser.add_argument('--apply', action='store_true', help='应用临时地址并验证 SDK 连接；默认只预览')
    parser.add_argument('--interface', help='仅探测指定物理有线接口，如 eth0；仍检查 ARP')
    parser.add_argument('--sn', action='append', default=[], help='仅匹配指定手套 SN，可重复')
    parser.add_argument('--prefix-length', type=int, choices=range(16, 31), default=24,
                        metavar='16..30', help='新地址的前缀长度，默认 24；SDK 扫描不提供设备掩码')
    args = parser.parse_args(argv)
    if os.geteuid() == 0:
        parser.error('请以普通用户运行；程序仅对 ARP 探测和添加地址调用 sudo。')
    manager = None
    try:
        network = LinuxNetwork()
        from wuji_sdk import SdkManager, DeviceType
        manager = SdkManager.instance()
        gloves = discover(manager, DeviceType.WujiGlove, args.sn)
        print('发现手套：' + ', '.join(f'{g.sn}={g.address}' for g in gloves), flush=True)
        # Authorize short-lived privileged network commands without running the SDK as root.
        subprocess.run(['sudo', '-v'], check=True)
        plan = build_plan(gloves, network, args.interface, args.prefix_length)
        for glove in gloves:
            print(f'{glove.sn} -> {plan.interfaces[glove.sn]}')
        for addition in plan.additions:
            print(f'添加计划：{addition.interface} + {addition.cidr}')
        if not plan.additions:
            print('已有同网段地址，无需添加。')
        if not args.apply:
            print('仅预览，未修改地址。应用并验证请加 --apply。')
            return 0
        apply_plan(gloves, plan, network, lambda: verify_gloves(manager, gloves))
        print('手套网络就绪；未使能任何机械手。临时地址在重启/网卡重连后可能失效，请重新运行此命令。')
        for addition in plan.additions:
            print(f'撤销本次地址：sudo ip address del {addition.cidr} dev {addition.interface}')
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print(f'手套网络准备失败：{exc or "用户中断"}', file=sys.stderr)
        return 1
    finally:
        if manager is not None:
            manager.disconnect_all()


if __name__ == '__main__':
    sys.exit(main())
