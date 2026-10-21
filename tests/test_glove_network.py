"""Network planning/rollback regression, including the reported wlan0 misroute.

No host addresses or physical devices are modified by these tests.
"""
import copy
import ipaddress as ip
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import glove_network as net


def row(name, address):
    a = ip.IPv4Interface(address)
    return dict(ifname=name, link_type='ether', flags=['UP', 'LOWER_UP'],
                addr_info=[dict(family='inet', local=str(a.ip), prefixlen=a.network.prefixlen)])


class Network:
    def __init__(self):
        self.rows = [row('eth0', '6.6.7.100/24'), row('eth10', '6.6.8.100/24'),
                     row('wlan0', '10.21.5.75/24')]
        self.owners = {('eth0', '192.168.1.100'), ('eth0', '192.168.1.101')}
        self.events = []
        self.extra_routes = []
        self.bad_route = False

    def addresses(self):
        return copy.deepcopy(self.rows)

    def wired(self, r):
        return r['ifname'].startswith('eth')

    def occupied(self, interface, host):
        return (interface, str(host)) in self.owners

    def route(self, host):
        if not self.bad_route:
            for name, address in net.local_addresses(self.rows):
                if host in address.network:
                    return dict(dev=name, prefsrc=str(address.ip))
        return dict(dev='wlan0', gateway='10.21.5.254', prefsrc='10.21.5.75')

    def routes(self):
        return [dict(dst='default', dev='wlan0', gateway='10.21.5.254'), *self.extra_routes]

    def add(self, a):
        self.events.append(('add', a))
        next(r for r in self.rows if r['ifname'] == a.interface)['addr_info'].extend(row(a.interface, a.cidr)['addr_info'])

    def remove(self, a):
        self.events.append(('del', a))
        r = next(r for r in self.rows if r['ifname'] == a.interface)
        r['addr_info'] = [v for v in r['addr_info'] if v['local'] != a.cidr.split('/')[0]]


def glove(sn='G1', address='192.168.1.100:50001'):
    return net.Glove(sn, address, ip.IPv4Address(address.split(':')[0]))


class PlanningTests(unittest.TestCase):
    def setUp(self):
        self.network = Network()
        self.gloves = [glove(), glove('G2', '192.168.1.101:50001')]

    def test_reported_wifi_route_adds_one_address_and_preserves_originals(self):
        original = self.network.addresses()
        plan = net.build_plan(self.gloves, self.network)
        self.assertEqual(plan.additions, [net.Addition('eth0', '192.168.1.200/24')])
        self.assertEqual(plan.interfaces, {'G1': 'eth0', 'G2': 'eth0'})
        self.assertEqual(self.network.events, [])  # preview is read-only
        verify = MagicMock()
        net.apply_plan(self.gloves, plan, self.network, verify)
        verify.assert_called_once()
        self.assertEqual(self.network.route(self.gloves[0].host)['dev'], 'eth0')
        for before, after in zip(original, self.network.rows):
            for address in before['addr_info']:
                self.assertIn(address, after['addr_info'])
        self.assertEqual(net.build_plan(self.gloves, self.network).additions, [])

    def test_conflicting_candidate_skipped(self):
        self.network.owners.add(('eth0', '192.168.1.200'))
        self.assertEqual(net.build_plan(self.gloves, self.network).additions[0].cidr, '192.168.1.201/24')

    def test_discovered_device_address_never_used_for_host(self):
        gs = [glove('G3', '192.168.1.200:50001')]
        self.network.owners.add(('eth0', '192.168.1.200'))
        self.assertEqual(net.build_plan(gs, self.network).additions[0].cidr, '192.168.1.201/24')

    def test_no_arp_response_fails_without_guessing(self):
        self.network.owners.clear()
        with self.assertRaisesRegex(RuntimeError, '无法唯一定位'):
            net.build_plan(self.gloves, self.network)
        self.assertFalse(self.network.events)

    def test_ambiguous_interfaces_require_explicit_choice(self):
        self.network.owners.add(('eth10', '192.168.1.100'))
        with self.assertRaisesRegex(RuntimeError, '无法唯一定位'):
            net.build_plan(self.gloves, self.network)
        self.assertEqual(net.build_plan(self.gloves, self.network, 'eth0').interfaces['G1'], 'eth0')

    def test_wifi_never_selected(self):
        with self.assertRaisesRegex(RuntimeError, '物理有线'):
            net.build_plan(self.gloves, self.network, 'wlan0')

    def test_other_interface_subnet_collision(self):
        self.network.rows.append(row('eth2', '192.168.1.2/24'))
        with self.assertRaisesRegex(RuntimeError, '重叠'):
            net.build_plan(self.gloves, self.network)

    def test_existing_specific_route_not_overridden(self):
        self.network.extra_routes.append(dict(dst='192.168.1.0/24', dev='wlan0', gateway='10.21.5.254'))
        with self.assertRaisesRegex(RuntimeError, '已有路由'):
            net.build_plan(self.gloves, self.network)

    def test_same_host_ip_collision(self):
        self.network.rows.append(row('eth3', '192.168.1.100/24'))
        with self.assertRaisesRegex(RuntimeError, '本机地址冲突'):
            net.build_plan(self.gloves, self.network)

    def test_second_glove_on_different_subnet(self):
        self.network.owners.add(('eth10', '192.168.2.100'))
        gs = [glove(), glove('G2', '192.168.2.100:50001')]
        plan = net.build_plan(gs, self.network)
        self.assertEqual(plan.additions, [net.Addition('eth0', '192.168.1.200/24'),
                                          net.Addition('eth10', '192.168.2.200/24')])

    def test_same_subnet_on_different_interfaces_rejected(self):
        self.network.owners.remove(('eth0', '192.168.1.101'))
        self.network.owners.add(('eth10', '192.168.1.101'))
        with self.assertRaisesRegex(RuntimeError, '网段重叠'):
            net.build_plan(self.gloves, self.network)

    def test_sdk_failure_rolls_back_only_added_addresses(self):
        original = self.network.addresses()
        plan = net.build_plan(self.gloves, self.network)
        with self.assertRaisesRegex(RuntimeError, 'SDK timeout'):
            net.apply_plan(self.gloves, plan, self.network, MagicMock(side_effect=RuntimeError('SDK timeout')))
        self.assertEqual(self.network.rows, original)
        self.assertEqual([event for event, _ in self.network.events], ['add', 'del'])

    def test_existing_address_never_removed_on_failure(self):
        self.network.add(net.Addition('eth0', '192.168.1.200/24'))
        self.network.events.clear()
        plan = net.build_plan(self.gloves, self.network)
        with self.assertRaises(RuntimeError):
            net.apply_plan(self.gloves, plan, self.network, MagicMock(side_effect=RuntimeError()))
        self.assertEqual(self.network.events, [])

    def test_wrong_route_after_add_rolls_back_without_sdk_connect(self):
        plan = net.build_plan(self.gloves, self.network)
        self.network.bad_route = True
        verify = MagicMock()
        with self.assertRaisesRegex(RuntimeError, '路由/源地址'):
            net.apply_plan(self.gloves, plan, self.network, verify)
        verify.assert_not_called()
        self.assertEqual([event for event, _ in self.network.events], ['add', 'del'])

    def test_candidate_race_detected_before_add(self):
        plan = net.build_plan(self.gloves, self.network)
        self.network.owners.add(('eth0', '192.168.1.200'))
        with self.assertRaisesRegex(RuntimeError, '已被占用'):
            net.apply_plan(self.gloves, plan, self.network, MagicMock())
        self.assertEqual(self.network.events, [])

    def test_second_add_failure_rolls_back_first(self):
        original = self.network.addresses()
        plan = net.Plan([net.Addition('eth0', '192.168.1.200/24'), net.Addition('eth10', '192.168.2.200/24')], {})
        add = self.network.add
        def fail_second(a):
            if a.interface == 'eth10':
                raise RuntimeError('add failed')
            add(a)
        self.network.add = fail_second
        with self.assertRaisesRegex(RuntimeError, 'add failed'):
            net.apply_plan(self.gloves, plan, self.network, MagicMock())
        self.assertEqual(original, self.network.rows)

    def test_interrupt_rolls_back(self):
        original = self.network.addresses()
        plan = net.build_plan(self.gloves, self.network)
        with self.assertRaises(KeyboardInterrupt):
            net.apply_plan(self.gloves, plan, self.network, MagicMock(side_effect=KeyboardInterrupt))
        self.assertEqual(original, self.network.rows)


class LinuxInterfaceTests(unittest.TestCase):
    def setUp(self):
        self.network = object.__new__(net.LinuxNetwork)
        self.network.ip = '/usr/sbin/ip'
        # Real Ubuntu iproute2 shape: -4 address omits link_type entirely.
        self.addr = [dict(ifindex=2, ifname='eth0', flags=['UP', 'LOWER_UP'],
                         addr_info=[dict(family='inet', local='6.6.7.100', prefixlen=24)])]
        self.links = [dict(ifindex=2, ifname='eth0', link_type='ether', flags=['UP', 'LOWER_UP']),
                      dict(ifindex=3, ifname='eth1', link_type='ether', flags=['UP', 'LOWER_UP'])]

    def output(self, command):
        return NS(stdout=json.dumps(self.addr if 'address' in command else self.links))

    def test_ipv4_address_output_missing_link_type_merges_link_information(self):
        with patch.object(net, 'run', side_effect=self.output):
            rows = self.network.addresses()
        self.assertEqual(rows[0].get('link_type'), 'ether')
        self.assertEqual(rows[0]['addr_info'][0]['local'], '6.6.7.100')

    def test_ethernet_without_ipv4_still_available(self):
        with patch.object(net, 'run', side_effect=self.output):
            rows = self.network.addresses()
        self.assertEqual(rows[1]['ifname'], 'eth1')
        self.assertEqual(rows[1]['addr_info'], [])

    def test_actual_wired_filter_accepts_merged_ethernet(self):
        with patch.object(net, 'run', side_effect=self.output), \
                patch.object(self.network, 'is_physical_wired', return_value=True):
            rows = self.network.addresses()
            self.assertTrue(self.network.wired(rows[0]))
            self.assertTrue(self.network.wired(rows[1]))

    def test_actual_wired_filter_still_rejects_invalid_links(self):
        with patch.object(self.network, 'is_physical_wired', return_value=True):
            for changes in [dict(link_type='loopback'), dict(flags=['UP']),
                            dict(flags=['LOWER_UP']), dict(master='br0')]:
                self.assertFalse(self.network.wired({**self.links[0], **changes}))
        with patch.object(self.network, 'is_physical_wired', return_value=False):
            self.assertFalse(self.network.wired(self.links[0]))


class DiscoveryTests(unittest.TestCase):
    def setUp(self):
        self.manager = MagicMock()
        self.manager.scan.return_value = [NS(sn='G1', device_type='glove', address='192.168.1.100:50001'),
                                          NS(sn='H1', device_type='hand1', address='usb:0483:2000:H1'),
                                          NS(sn='H2', device_type='hand2', address='6.6.7.110:7447')]

    def test_only_gloves_are_selected(self):
        self.assertEqual(net.discover(self.manager, 'glove', []), [glove()])
        self.manager.connect.assert_not_called()

    def test_missing_selected_serial(self):
        with self.assertRaises(RuntimeError):
            net.discover(self.manager, 'glove', ['MISSING'])

    def test_duplicate_ip_rejected(self):
        self.manager.scan.return_value.append(NS(sn='G2', device_type='glove', address='192.168.1.100:50001'))
        with self.assertRaisesRegex(RuntimeError, '相同 IP'):
            net.discover(self.manager, 'glove', [])

    def test_invalid_address_rejected(self):
        self.manager.scan.return_value[0].address = 'garbage'
        with self.assertRaisesRegex(RuntimeError, '地址不支持'):
            net.discover(self.manager, 'glove', [])

    def test_sdk_verification_reads_skeleton_without_motor_calls(self):
        device = MagicMock()
        device.serial_number = 'G1'
        frame = NS(joints=[NS(pose=NS(position=[.1, .2, .3])) for _ in range(21)])
        device.hand_skeleton.return_value.subscribe.return_value.recv.return_value = frame
        self.manager.connect.return_value = device
        with patch.dict(sys.modules, {'wuji_sdk': NS(ConnectOptions=lambda **kw: kw)}):
            net.verify_gloves(self.manager, [glove()])
        self.assertEqual(self.manager.connect.call_args.kwargs['address'], '192.168.1.100:50001')
        device.enable.assert_not_called()
        device.ip.assert_not_called()
        device.hand_skeleton.return_value.subscribe.return_value.close.assert_called_once()

    def test_wrong_serial_fails_before_subscription(self):
        device = MagicMock(serial_number='WRONG')
        self.manager.connect.return_value = device
        with patch.dict(sys.modules, {'wuji_sdk': NS(ConnectOptions=lambda **kw: kw)}):
            with self.assertRaisesRegex(RuntimeError, '其他 SN'):
                net.verify_gloves(self.manager, [glove()])
        device.hand_skeleton.assert_not_called()
        self.manager.disconnect_all.assert_called()

    def test_arp_command_errors_are_not_free_addresses(self):
        network = object.__new__(net.LinuxNetwork)
        network.arping = '/usr/bin/arping'
        for status, stderr in [(2, 'permission denied'), (1, 'failure')]:
            with patch.object(net, 'run', return_value=NS(returncode=status, stderr=stderr, stdout='')):
                with self.assertRaisesRegex(RuntimeError, 'ARP 检查失败'):
                    network.occupied('eth0', '192.168.1.200')


if __name__ == '__main__':
    unittest.main()
