"""Real ip/arping regression in disposable user+network namespaces only.

Usage: unshare -Urn python3 tests/test_glove_network_namespace.py /path/to/arping
No SDK or physical devices. Only the sysfs physical-device check is overridden
for synthetic veths; production link/address parsing and wired filtering run.
"""
import os
import json
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from glove_network import LinuxNetwork, Glove, build_plan, apply_plan, verify_routes
from ipaddress import IPv4Address


def command(*args):
    return subprocess.run(args, check=True, capture_output=True, text=True).stdout


def main():
    # Never construct the topology in the caller's host network namespace.
    assert os.geteuid() == 0, 'Run inside unshare -Urn'
    uid_map = Path('/proc/self/uid_map').read_text().split()
    assert len(uid_map) == 3 and uid_map[0] == '0' and uid_map[1] != '0' and uid_map[2] == '1', 'Expected unprivileged user namespace'
    links = json.loads(command('/usr/sbin/ip', '-j', 'link', 'show'))
    assert [r['ifname'] for r in links] == ['lo'], 'Expected an empty network namespace'
    network = object.__new__(LinuxNetwork)
    network.ip = '/usr/sbin/ip'
    network.arping = str(Path(sys.argv[1]).resolve())
    network.is_physical_wired = lambda name: name == 'eth0'  # veth instead of physical NIC
    peer = subprocess.Popen(['unshare', '-n', 'sleep', '120'])
    try:
        for _ in range(100):
            if os.readlink(f'/proc/{peer.pid}/ns/net') != os.readlink('/proc/self/ns/net'):
                break
            time.sleep(.01)
        else:
            raise RuntimeError('Peer namespace did not start')
        def peer_command(*args):
            return command('nsenter', '-t', str(peer.pid), '-n', *args)
        command(network.ip, 'link', 'add', 'eth0', 'type', 'veth', 'peer', 'name', 'gloves')
        command(network.ip, 'link', 'set', 'gloves', 'netns', str(peer.pid))
        command(network.ip, 'address', 'add', '6.6.7.100/24', 'dev', 'eth0')
        command(network.ip, 'link', 'set', 'eth0', 'up')
        peer_command(network.ip, 'link', 'set', 'gloves', 'up')
        for host in ['192.168.1.100', '192.168.1.101', '192.168.1.200']:
            peer_command(network.ip, 'address', 'add', host + '/24', 'dev', 'gloves')
        command(network.ip, 'link', 'add', 'wlan0', 'type', 'dummy')
        command(network.ip, 'link', 'set', 'wlan0', 'up')
        command(network.ip, 'address', 'add', '10.21.5.75/24', 'dev', 'wlan0')
        command(network.ip, 'route', 'add', 'default', 'via', '10.21.5.254', 'dev', 'wlan0')
        gloves = [Glove('G1', '192.168.1.100:50001', IPv4Address('192.168.1.100')),
                  Glove('G2', '192.168.1.101:50001', IPv4Address('192.168.1.101'))]
        before = network.addresses()
        assert [r['ifname'] for r in before if network.wired(r)] == ['eth0']
        assert network.route(gloves[0].host)['dev'] == 'wlan0'
        plan = build_plan(gloves, network)
        assert len(plan.additions) == 1 and plan.additions[0].cidr == '192.168.1.201/24', plan
        assert network.addresses() == before, 'Preview mutated addresses'
        def verify():
            for glove in gloves:
                command('ping', '-c', '1', '-W', '1', str(glove.host))
        apply_plan(gloves, plan, network, verify)
        verify_routes(gloves, plan, network)
        assert build_plan(gloves, network).additions == [], 'Not idempotent'
        assert '10.21.5.254' in command(network.ip, 'route', 'show', 'default')
        assert '6.6.7.100' in command(network.ip, '-4', 'address', 'show', 'dev', 'eth0')
        network.remove(plan.additions[0])
        def fail():
            raise RuntimeError('simulated SDK failure')
        try:
            apply_plan(gloves, plan, network, fail)
        except RuntimeError as exc:
            assert str(exc) == 'simulated SDK failure', exc
        else:
            raise AssertionError('Expected failure')
        assert network.route(gloves[0].host)['dev'] == 'wlan0', 'Rollback did not restore route'
        assert not any('192.168.1.201' == a.get('local') for r in network.addresses() for a in r['addr_info'])

        # Model the observed silence by dropping zero-source and old-subnet
        # requests at the virtual peer. Do not infer the glove's firmware cause.
        peer_command(network.ip, 'address', 'del', '192.168.1.200/24', 'dev', 'gloves')
        peer_command('tc', 'qdisc', 'add', 'dev', 'gloves', 'clsact')
        for priority, source in [('1', '0.0.0.0/32'), ('2', '6.6.7.0/24')]:
            peer_command('tc', 'filter', 'add', 'dev', 'gloves', 'ingress', 'protocol', 'arp',
                         'pref', priority, 'flower', 'arp_sip', source, 'action', 'drop')
        assert not network.occupied('eth0', gloves[0].host), 'Expected zero-source ARP to be ignored'
        cross = subprocess.run([network.arping, '-I', 'eth0', '-s', '6.6.7.100',
                                '-c', '1', '-w', '1', str(gloves[0].host)], capture_output=True, text=True)
        assert cross.returncode == 1 and not cross.stderr.strip(), cross
        fallback = build_plan(gloves, network)
        assert fallback.additions[0].cidr == '192.168.1.200/24'
        apply_plan(gloves, fallback, network, verify)
        assert build_plan(gloves, network).additions == [], 'Fallback is not idempotent'
        network.remove(fallback.additions[0])
        try:
            apply_plan(gloves, fallback, network, fail)
        except RuntimeError as exc:
            assert str(exc) == 'simulated SDK failure', exc
        else:
            raise AssertionError('Fallback skipped verification')
        assert network.route(gloves[0].host)['dev'] == 'wlan0', 'Fallback rollback failed'
        assert '6.6.7.100' in command(network.ip, '-4', 'address', 'show', 'dev', 'eth0')
        print('PASS: real link/address parsing and wired filtering, ARP interface detection, occupied .200 skipped, .201 address/route, ping both peers, '
              'idempotency, original IP/default gateway preservation, failure rollback; '
              'zero-source/cross-subnet ARP silence followed by successful same-subnet setup and fallback rollback; '
              'isolated namespaces only')
    finally:
        peer.terminate()
        peer.wait(timeout=5)


if __name__ == '__main__':
    main()
