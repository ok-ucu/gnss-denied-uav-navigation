"""VIO source <-> transport latency contract: one definition, pinned to both sides."""
import inspect
import json
from pathlib import Path
import re
import sys
import unittest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import vio_latency_contract as c

EVIDENCE = HERE/'debug_evidence'


class ContractTests(unittest.TestCase):
    def test_source_limit_is_derived_from_transport_limits(self):
        self.assertAlmostEqual(c.SOURCE_PUBLICATION_MAX_AGE_S, c.TRANSPORT_FRESHNESS_S-c.MAPPING_UNCERTAINTY_MAX_S
                               - c.DELIVERY_RESERVE_S-c.PAIR_PROCESSING_RESERVE_S, places=9)
        self.assertAlmostEqual(c.SOURCE_PUBLICATION_MAX_AGE_S, .157, places=9)
        self.assertLess(c.SOURCE_PUBLICATION_MAX_AGE_S, .18)   # tighter than before, never looser

    def test_worst_case_published_sample_still_has_pairing_budget(self):
        # Published at the limit, delivered at the reserve, worst declared mapping uncertainty.
        budget = c.TRANSPORT_FRESHNESS_S-(c.SOURCE_PUBLICATION_MAX_AGE_S+c.DELIVERY_RESERVE_S)-c.MAPPING_UNCERTAINTY_MAX_S
        self.assertGreaterEqual(budget+1e-12, c.PAIR_PROCESSING_RESERVE_S)

    def test_transport_limits_unchanged_and_match_contract(self):
        from physics_clock_mapping import AppliedClockMapping
        default = inspect.signature(AppliedClockMapping.__init__).parameters['max_uncertainty_ns'].default
        self.assertEqual(default, round(c.MAPPING_UNCERTAINTY_MAX_S*1e9))
        lifecycle = (HERE/'navigation_lifecycle.py').read_text()
        ns = '{:_}'.format(round(c.TRANSPORT_FRESHNESS_S*1e9))
        self.assertIn('-5_000_000<=freshness_clock-ns<='+ns, lifecycle)
        self.assertIn('host_ns+max(0,'+ns+'-max(0,freshness_clock-ns)', lifecycle)

    def test_source_uses_the_contract_and_no_private_limit(self):
        source = (HERE/'run_vio_stream.py').read_text()
        self.assertIn('from vio_latency_contract import SOURCE_PUBLICATION_MAX_AGE_S as PUBLICATION_MAX_AGE_S', source)
        self.assertIn('limit=PUBLICATION_MAX_AGE_S', source)
        self.assertIn('Admission(PUBLICATION_MAX_AGE_S)', source)
        self.assertIsNone(re.search(r'limit=\.18\b|publication_max_age_sim_s=\.18\b', source))

    def test_transport_reports_reserve_violations(self):
        transport = (HERE/'phase1_transport.py').read_text()
        self.assertIn("event='latency_contract_exceeded'", transport)
        self.assertIn('contract.DELIVERY_RESERVE_S', transport)
        self.assertIn('contract.PAIR_PROCESSING_RESERVE_S', transport)


@unittest.skipUnless((EVIDENCE/'20261117-flight4414cb818f2a/flight-source/stream.jsonl').is_file(), 'Preserved evidence not in checkout')
class ReservesCoverRecordedFlights(unittest.TestCase):
    def runs(self):
        for name, transport in (('20261117-flight4414cb818f2a', 'flight-transport'), ('20261117-flight6f32c08e7aaf', '.')):
            d = EVIDENCE/name
            if (d/'flight-source/stream.jsonl').is_file():
                yield d, d/transport

    def test_delivery_and_pairing_within_reserves(self):
        for run, transport in self.runs():
            stages = [json.loads(line) for line in (transport/'stages.jsonl').read_text().splitlines()]
            pair = {r['source_ns']: r['age_sim_ns'] for r in stages
                    if r['stage'] == 'pair_input_age' and r.get('age_sim_ns') is not None}
            stream = [json.loads(line) for line in (run/'flight-source/stream.jsonl').read_text().splitlines()]
            delivery = [pair[round(r['t']*1e9)]/1e9-r['publication_clock_check']['simulation_age_s'] for r in stream
                        if r.get('published') and r.get('publication_clock_check') and round(r['t']*1e9) in pair]
            pairing = [(r['host_ns']-r['approved_host_ns'])/1e9 for r in stages if r['stage'] == 'permission_created']
            self.assertTrue(delivery and pairing, run)
            self.assertLessEqual(max(delivery), c.DELIVERY_RESERVE_S+1e-9, run)
            self.assertLessEqual(max(pairing), c.PAIR_PROCESSING_RESERVE_S, run)

    def test_mapping_uncertainty_within_declared_limit(self):
        mapping = EVIDENCE/'20261117-flight4414cb818f2a/flight-transport/mapping.jsonl'
        worst = max(json.loads(line)['uncertainty_ns'] for line in mapping.read_text().splitlines())
        self.assertLessEqual(worst, c.MAPPING_UNCERTAINTY_MAX_S*1e9)


if __name__ == '__main__':
    unittest.main()
