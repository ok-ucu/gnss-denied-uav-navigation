"""Offline launch/probe checks; live SITL evidence is stored separately."""
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import verify_sitl

ROOT = Path(__file__).resolve().parents[1]


class SetupTests(unittest.TestCase):
    def test_external_nav_copy_matches_expected_source(self):
        expected = {'EK3_SRC1_POSXY':6,'EK3_SRC1_VELXY':6,'EK3_SRC1_POSZ':1,
                    'EK3_SRC1_VELZ':6,'EK3_SRC1_YAW':1,'VISO_TYPE':1,'GPS1_TYPE':0}
        actual = dict(line.split() for line in (ROOT/'sitl_ext_nav.parm').read_text().splitlines()
                      if line.strip() and not line.startswith('#'))
        self.assertEqual({k:int(v) for k,v in actual.items()},expected)
        self.assertEqual(verify_sitl.EXPECTED['GPS_TYPE'],0)

    def test_same_or_invalid_ports_rejected_before_launch(self):
        for covariance, monitor in [('14552','14552'),('bad','14552'),('70000','14552')]:
            environment = dict(os.environ,COVARIANCE_PORT=covariance,MONITOR_PORT=monitor)
            result = subprocess.run(['bash',str(ROOT/'start_sitl.sh')],env=environment,
                                    capture_output=True,text=True)
            self.assertEqual(result.returncode,2)

    def test_probe_reports_two_links_without_claiming_local_position(self):
        def heartbeat():
            return NS(get_type=lambda:'HEARTBEAT',autopilot=3,base_mode=0,
                      get_srcSystem=lambda:1,get_srcComponent=lambda:1,
                      get_msgbuf=lambda:bytes([0xFD]))
        def message(kind, **values):
            return NS(get_type=lambda:kind,get_srcSystem=lambda:1,
                      get_srcComponent=lambda:1,**values)
        clients=[]
        for _ in range(2):
            queue=[heartbeat(),heartbeat(),message('ATTITUDE'),message('EKF_STATUS_REPORT')]
            queue += [message('PARAM_VALUE',param_id=k,param_value=v)
                      for k,v in verify_sitl.EXPECTED.items()]
            client=Mock()
            # Bind each lambda to its own queue.
            client.recv_match.side_effect=lambda queue=queue,**kw:queue.pop(0) if queue else None
            clients.append(client)
        with patch.dict(sys.modules,{'pymavlink':NS(mavutil=Mock())}), \
             patch.object(verify_sitl,'connection',side_effect=clients):
            results=verify_sitl.verify(['first','second'],timeout=1)
        self.assertTrue(all(r['passed'] for r in results))
        self.assertTrue(all(not r['local_position_available'] for r in results))
        for client in clients:
            client.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
