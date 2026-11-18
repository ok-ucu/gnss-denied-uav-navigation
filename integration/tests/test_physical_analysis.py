"""Known synthetic trajectories/timing; not measured hardware performance."""
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import tempfile
import unittest

from analyze_physical_vio import (analyze, json_rows, latency, raw_time, trajectory, write_outputs,
                                  se3_translational_rpe, world_displacement_error)
from analyze_trajectories import compare, load, timestamp


class PhysicalAnalysisTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name)
        self.start = 1_800_000_000_000_000_001
        proof = b'SYNTHETIC analytic clock fixture; NOT physical clock evidence\n'
        (self.base/'proof.txt').write_bytes(proof)
        self.proof_hash = hashlib.sha256(proof).hexdigest()
        self.write_csv('ref.csv', [[self.start+i*100_000_000, i, i/2, 0.] for i in range(4)])
        self.write_csv('vio.csv', [[self.start+i*100_000_000, i+1, i/2+2, 0.] for i in range(4)])
        def spec(path, session):
            return dict(path=path, format='enu-csv', position_unit='m', time_unit='ns', time_column='timestamp',
                clock_domain='SYNTHETIC_NS', session=session, reference_id='synthetic-common-origin',
                mapping=dict(reviewed=True, reviewer='synthetic-test-only', evidence_path='proof.txt',
                    evidence_sha256=self.proof_hash, source_clock='SYNTHETIC_NS', source_epoch=session,
                    target_clock='SYNTHETIC_COMMON_NS', target_epoch='test-only', uncertainty_ns=1,
                    max_uncertainty_ns=100, max_anchor_age_ns=1000,
                    anchors=[dict(source_ns=self.start+i*150_000_000, target_ns=1+i*150_000_000,
                                  host_ns=1+i*150_000_000) for i in range(3)]))
        self.c = dict(reference=spec('ref.csv', 'reference'), candidate=spec('vio.csv', 'vio'),
            max_interpolation_gap_s=.15, rpe_delta_samples=1, max_rpe_interval_s=.15,
            source_availability_limit_ns=500_000_000)

    def write_csv(self, path, rows, header=('timestamp', 'east', 'north', 'up')):
        with (self.base/path).open('w', newline='') as stream:
            writer = csv.writer(stream); writer.writerow(header); writer.writerows(rows)

    def test_known_ate_and_translation_rpe_no_hidden_alignment(self):
        r = analyze(self.c, self.base)
        self.assertEqual(r['analysis_status'], 'PASS')
        self.assertEqual(r['status'], 'NOT VERIFIED')
        self.assertAlmostEqual(r['metrics']['spatial_rmse_m'], math.sqrt(5))
        self.assertEqual(r['metrics']['spatial_drift_m'], 0)
        self.assertEqual(r['world_displacement_error']['rmse_m'], 0)
        self.assertEqual(r['world_displacement_error']['pairs'], 3)
        self.assertEqual(r['rpe']['status'], 'NOT VERIFIED')
        self.assertIsNone(r['rpe']['rmse_m'])
        self.assertFalse(r['execution_allowed']); self.assertFalse(r['forwarding_enabled'])

    def test_drift_and_rpe_with_known_scale_error(self):
        self.write_csv('vio.csv', [[self.start+i*100_000_000, 2*i, i/2, 0.] for i in range(4)])
        r = analyze(self.c, self.base)
        self.assertEqual(r['analysis_status'], 'PASS')
        self.assertEqual(r['metrics']['spatial_drift_m'], 3)
        self.assertEqual(r['world_displacement_error']['rmse_m'], 1)

    def test_missing_unreviewed_or_corrupt_clock_proof_never_passes(self):
        for edit in (lambda c: c['candidate'].pop('mapping'),
                     lambda c: c['candidate']['mapping'].update(reviewed=False),
                     lambda c: c['candidate']['mapping'].update(source_epoch='other'),
                     lambda c: c['candidate']['mapping'].update(target_epoch='other'),
                     lambda c: c['candidate']['mapping'].update(evidence_sha256='0'*64),
                     lambda c: c['candidate']['mapping'].update(uncertainty_ns=10000)):
            c = copy.deepcopy(self.c); edit(c)
            r = analyze(c, self.base)
            self.assertNotEqual(r['analysis_status'], 'PASS'); self.assertNotEqual(r['status'], 'PASS')

    def test_origin_mismatch_refuses_no_yaw_or_scale_fitting(self):
        self.c['candidate']['reference_id'] = 'unknown'
        self.assertNotEqual(analyze(self.c, self.base)['analysis_status'], 'PASS')

    def test_raw_order_resets_duplicate_nan_and_wrong_time_unit(self):
        for stamps in ([self.start, self.start], [self.start+1, self.start]):
            self.write_csv('vio.csv', [[t, 1., 2., 3.] for t in stamps])
            self.assertEqual(analyze(self.c, self.base)['analysis_status'], 'FAIL')
        self.write_csv('vio.csv', [[self.start, float('nan'), 0, 0], [self.start+1, 0, 0, 0]])
        self.assertEqual(analyze(self.c, self.base)['analysis_status'], 'FAIL')
        # Restore finite positions: units must fail for their own reason.
        self.write_csv('vio.csv', [[self.start+i*100_000_000, i, 0., 0.] for i in range(4)])
        self.c['candidate']['time_unit'] = 's'
        failed = analyze(self.c, self.base)
        self.assertEqual(failed['analysis_status'], 'FAIL')
        self.assertIn('Acquisition outside calibrated clock interval', failed['reason'])
        self.assertEqual(raw_time('1800000000000000001', 'ns'), self.start)
        for value in ('NaN', 'Infinity', '-1', '.5'):
            with self.assertRaises(ValueError): raw_time(value, 'ns')

    def test_raw_pi_logs_preserve_session_stamps_and_no_unready_publications(self):
        spec = dict(self.c['candidate'], path='pi.jsonl', format='pi-jsonl', clock_domain='ros-system-unix-ns')
        rows = [dict(event='frame', session='vio', stamp_ns=self.start+i, p=[1.,2.,3.], ready=True, published=True) for i in range(3)]
        path = self.base/'pi.jsonl'
        path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
        t, e = trajectory(spec, self.base)
        self.assertEqual(t[0], (self.start, (2.,1.,-3.))); self.assertEqual(e['published'], 3)
        for key, val in (('session', 'wrong'), ('ready', False), ('stamp_ns', self.start)):
            changed = copy.deepcopy(rows); changed[1][key] = val
            path.write_text(''.join(json.dumps(r)+'\n' for r in changed))
            with self.assertRaises(ValueError): trajectory(spec, self.base)

    def test_dataflash_adapter_and_aiding_not_acceptance_proof(self):
        (self.base/'synthetic.BIN').write_bytes(b'SYNTHETIC DECODER FIXTURE ONLY')
        spec = dict(self.c['candidate'], path='synthetic.BIN', format='dataflash', time_unit='us',
                    clock_domain='FCU_BOOT_TIME_US', core=0)
        records = [dict(mavpackettype='XKF1', TimeUS=i*1000, C=0, PN=i, PE=2*i, PD=-i) for i in range(3)]
        records += [dict(mavpackettype='MSG', TimeUS=3000, Message='EKF3 is using external nav data')]
        rows, extra = trajectory(spec, self.base, decoder=lambda p: records)
        self.assertEqual(rows[1], (1_000_000, (2,1,1)))
        self.assertEqual(len(extra['aiding_messages']), 1)
        self.assertTrue(extra['fusion'].startswith('NOT VERIFIED'))
        records[2]['TimeUS'] = 1
        with self.assertRaises(ValueError): trajectory(spec, self.base, decoder=lambda p: records)

    def test_gps_requires_datum_and_is_not_automatically_gt(self):
        self.write_csv('gps.csv', [[1, 49.,24.,300.], [2, 49.,24.,301.]], ('timestamp','lat','lon','alt'))
        spec = dict(self.c['reference'], path='gps.csv', format='gps-csv', time_unit='s',
                    altitude_datum='WGS84_ellipsoid_m', geodetic_origin=[49.,24.,300.])
        rows, extra = trajectory(spec, self.base)
        self.assertAlmostEqual(rows[1][1][2], 1, places=6)
        self.assertIn('NOT VERIFIED', extra['reference_accuracy'])
        spec['altitude_datum'] = 'MSL'
        with self.assertRaises(ValueError): trajectory(spec, self.base)

    def test_dropouts_reported_not_silently_interpolated(self):
        c = copy.deepcopy(self.c)
        c['source_availability_limit_ns'] = 50_000_000
        r = analyze(c, self.base)
        self.assertEqual(len(r['continuity']['dropout_intervals_s']), 3)
        self.assertIn('leading/trailing', r['continuity']['scope'])

    def test_latency_exact_boundaries_no_fcu_or_wire_inference(self):
        base = dict(session='A', epoch=0, acquisition_ns=100, host_boot_id='boot',
                    clock_domain='HOST_MONOTONIC_NS', time_namespace='time:[1]', clock_instance='clock-A')
        a, b = dict(base, event='publish', host_ns=1000), dict(base, event='forward', host_ns=1001000, boundary='os_write_complete')
        proof = dict(kind='shared-monotonic-instance', verified=True, reviewer='synthetic-only',
                     host_boot_id='boot', time_namespace='time:[1]', clock_instance='clock-A', valid_from_ns=0, valid_to_ns=2_000_000)
        r = latency([a,b], proof); self.assertEqual(r['pairs'], 1)
        self.assertEqual(r['publish_to_forward_ms']['mean'], 1.)
        self.assertIn('NOT VERIFIED', r['fcu_reception_latency'])
        self.assertEqual(latency([a])['unpaired_samples'], 1)
        for bad in (dict(b, host_ns=999), dict(b, host_boot_id='other'), dict(b, boundary='ros_enqueue')):
            with self.assertRaises(ValueError): latency([a, bad])
        with self.assertRaises(ValueError): latency([a,a,b])
        self.assertEqual(latency([a,b])['status'], 'NOT VERIFIED')
        self.assertIsNone(latency([a,b])['publish_to_forward_ms'])
        self.assertEqual(latency([a,b])['unverified_pairs'], 1)
        for bad in (dict(b, time_namespace='time:[2]'), dict(b, clock_instance='other')):
            with self.assertRaises(ValueError): latency([a, bad], proof)
        for change in (dict(verified=False), dict(valid_to_ns=1000), dict(host_boot_id='other')):
            with self.assertRaises(ValueError): latency([a,b], dict(proof, **change))

    def test_json_csv_outputs_and_portable_input_proof(self):
        r = analyze(self.c, self.base); output = self.base/'out'
        write_outputs(r, output)
        self.assertEqual(json.loads((output/'report.json').read_text())['analysis_status'], 'PASS')
        self.assertEqual(len((output/'samples.csv').read_text().splitlines()), 5)
        for name in ('ref.csv', 'vio.csv', 'proof.txt'): shutil.copyfile(self.base/name, output/name)
        self.assertEqual(analyze(self.c, output)['metrics']['spatial_rmse_m'], r['metrics']['spatial_rmse_m'])

    def test_partial_json_and_missing_files_fail(self):
        p = self.base/'partial.jsonl'; p.write_text('{"event":')
        with self.assertRaises(ValueError): json_rows(p)
        self.c['candidate']['path'] = 'absent.csv'
        self.assertEqual(analyze(self.c, self.base)['analysis_status'], 'FAIL')

    def test_reused_legacy_rmse_finite_settings_and_no_clock_assumptions(self):
        rows = [(0., (0.,0.,0.)), (1.,(1.,0.,0.))]
        with self.assertWarns(UserWarning), self.assertRaises(ValueError): compare(rows, rows)
        for gap in (float('nan'), float('inf'), 0.):
            with self.assertRaises(ValueError): compare(rows, rows, True, gap)
        with self.assertRaises(ValueError): load(self.base/'ref.csv', offset=float('nan'))

    def test_standard_rpe_detects_constant_90_degree_orientation_error(self):
        path = [(0., (0.,0.,0.)), (1., (1.,0.,0.))]
        metrics = compare(path, path, True, 2)
        identity = [0.,0.,0.,1.]; yaw90 = [0.,0.,math.sqrt(.5),math.sqrt(.5)]
        self.assertEqual(world_displacement_error(metrics, 1, 2)['rmse_m'], 0)
        unverified = se3_translational_rpe(metrics, [identity]*2, [yaw90]*2, 1, 2)
        self.assertEqual(unverified['status'], 'NOT VERIFIED')
        actual = se3_translational_rpe(metrics, [identity]*2, [yaw90]*2, 1, 2, verified=True)
        self.assertAlmostEqual(actual['rmse_m'], math.sqrt(2))
        same = se3_translational_rpe(metrics, [yaw90]*2, [yaw90]*2, 1, 2, verified=True)
        self.assertAlmostEqual(same['rmse_m'], 0)
        with self.assertRaises(ValueError):
            se3_translational_rpe(metrics, [identity]*2, [[0,0,0,2]]*2, 1, 2, verified=True)
        self.assertEqual(se3_translational_rpe(metrics, [identity]*2, [yaw90]*2, 2, 2, verified=True)['status'], 'NOT VERIFIED')

    def add_attitudes(self):
        rows = [dict(time_ns=1+i*100_000_000, reference_xyzw=[0,0,0,1],
                     candidate_xyzw=[0,0,0,1]) for i in range(4)]
        path = self.base/'orientations.jsonl'
        path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
        self.c['orientation_pairs'] = dict(path=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            verified=True, reviewer='synthetic-only', body_frame='shared-body',
            convention='body-to-world-ENU-xyzw', reference_id='synthetic-common-origin',
            reference_session='reference', candidate_session='vio',
            target_clock='SYNTHETIC_COMMON_NS', target_epoch='test-only',
            reference_sha256=hashlib.sha256((self.base/'ref.csv').read_bytes()).hexdigest(),
            candidate_sha256=hashlib.sha256((self.base/'vio.csv').read_bytes()).hexdigest())

    def test_end_to_end_orientation_proof_is_bound_to_sources_and_exact_times(self):
        self.add_attitudes()
        r = analyze(self.c, self.base)
        self.assertEqual(r['analysis_status'], 'PASS')
        self.assertEqual(r['rpe']['status'], 'PASS')
        for key, value in (('verified', False), ('candidate_session', 'other'), ('candidate_sha256', '0'*64),
                           ('target_epoch','wrong'), ('convention','NED'), ('sha256','0'*64)):
            c = copy.deepcopy(self.c); c['orientation_pairs'][key] = value
            self.assertEqual(analyze(c, self.base)['analysis_status'], 'FAIL')
        p = self.base/'orientations.jsonl'; rows = json_rows(p)
        rows[1]['time_ns'] += 1
        p.write_text(''.join(json.dumps(r)+'\n' for r in rows))
        self.c['orientation_pairs']['sha256'] = hashlib.sha256(p.read_bytes()).hexdigest()
        self.assertIn('exact-time orientation', analyze(self.c, self.base)['reason'])

    def test_full_reference_path_21_m_not_1_m_and_gaps_are_explicit(self):
        truth = [(0., (0.,0.,0.)), (.1, (10.,0.,0.)), (.2, (0.,0.,0.)), (.3, (1.,0.,0.))]
        vio = [(0., (0.,0.,0.)), (.2, (0.,0.,0.)), (.3, (2.,0.,0.))]
        r = compare(truth, vio, True, .15)
        self.assertEqual(r['truth_path_m'], 21.)
        self.assertAlmostEqual(r['spatial_drift_percent'], 100/21)
        broken = compare(truth, vio, True, .05)
        self.assertIsNone(broken['truth_path_m'])
        self.assertIsNone(broken['spatial_drift_percent'])
        self.assertEqual(broken['reference_path_status'], 'NOT VERIFIED')
        self.assertTrue(broken['reference_path_gaps_ns'])

    def test_integer_nanoseconds_preserved_in_validation_alignment_and_iso(self):
        self.assertNotEqual(timestamp(str(self.start), 'ns'), timestamp(str(self.start+1), 'ns'))
        self.write_csv('precise.csv', [[self.start,0,0,0], [self.start+1,1,0,0]])
        rows = load(self.base/'precise.csv', unit='ns')
        self.assertEqual([r[0] for r in rows], [self.start, self.start+1])
        r = compare(rows, rows, True, .1, time_unit='ns')
        self.assertEqual(r['matched_times_ns'], [self.start, self.start+1])
        self.assertEqual(r['duration_s'], 1e-9)
        self.assertEqual(timestamp('2027-01-15T08:00:00.000000001Z', 'iso')-
                         timestamp('2027-01-15T08:00:00Z', 'iso'), 1)
        for value in (float(self.start), True, '1.5'):
            with self.assertRaises(ValueError): timestamp(value, 'ns')

    def test_latency_file_requires_hashed_clock_proof(self):
        event = dict(session='A', epoch=0, acquisition_ns=100, host_boot_id='boot',
                     clock_domain='HOST_MONOTONIC_NS', time_namespace='time:[1]', clock_instance='clock-A')
        rows = [dict(event,event='publish',host_ns=1000), dict(event,event='forward',host_ns=2000,boundary='os_write_complete')]
        p = self.base/'latency.jsonl'; p.write_text(''.join(json.dumps(r)+'\n' for r in rows))
        self.c['latency_jsonl'] = p.name
        self.assertEqual(analyze(self.c, self.base)['latency']['status'], 'NOT VERIFIED')
        self.c['latency_clock_proof'] = dict(kind='shared-monotonic-instance', verified=True,
            reviewer='synthetic-only', host_boot_id='boot',time_namespace='time:[1]',clock_instance='clock-A',
            valid_from_ns=0, valid_to_ns=10000,evidence_path='proof.txt',evidence_sha256=self.proof_hash,
            trace_sha256=hashlib.sha256(p.read_bytes()).hexdigest())
        self.assertEqual(analyze(self.c, self.base)['latency']['status'], 'PASS')
        self.c['latency_clock_proof']['evidence_sha256'] = '0'*64
        self.assertIn('clock evidence/trace hash', analyze(self.c, self.base)['reason'])

    def test_plot_uses_only_supplied_data_if_matplotlib_available(self):
        try: import matplotlib
        except ImportError: self.skipTest('Matplotlib absent; actual plotting NOT TESTED')
        output = self.base/'plots'; r = analyze(self.c, self.base)
        write_outputs(r, output, True)
        self.assertTrue((output/'xy.png').is_file()); self.assertTrue((output/'xyz.png').is_file())


if __name__ == '__main__': unittest.main()
