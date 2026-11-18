"""IMU propagation step shared by the frame path and between-frame pre-integration.

Why (run 20261117-flight4414cb818f2a): each frame integrated its ~100 IMU
samples (1 kHz) synchronously on the single frame thread, 23-70 ms per frame,
although those samples had arrived up to ~230 ms earlier while the thread was
idle. In CLIMB processing rose to 111-168 ms, the thread stopped going idle
between frames (7-13 ms instead of ~124 ms), every frame waited 54-108 ms in
the camera slot, and frame 37.9 s was published 175 ms old -> zero transport
budget -> 'permission expired during conversion'.

The arithmetic is unchanged: the same per-sample EKF prediction, in the same
order, with the same split at the frame time. Only WHEN it runs changes:
samples strictly before the next expected frame are integrated between frames;
the remainder (normally 0-2 samples) and the partial step to the frame time
stay on the frame path. Ordering with the frame's EKF updates is preserved
because pre-integration never passes a frame that may still be processed (see
preintegration_bound), and that frame's updates happen at its own stamp.
"""
import math

# Sim camera stamps lie on an exact grid; this only keeps a sample stamped
# exactly at the next frame time for the frame path, as before.
STAMP_EPSILON_S = 1e-6
# Samples integrated per idle tick. The capture loop polls the camera slot
# between ticks, so a frame never waits behind more than one chunk (~2-3 ms on
# the flight host at 0.25-0.35 ms/sample). 10 per ~4.5 ms loop is ~2x the 1 kHz
# IMU rate, so a post-frame backlog is still cleared well before the next frame.
TICK_CHUNK = 10
# EKF propagation step. The flight model delivers the ArduPilot IMU at 1 kHz;
# the VIO was developed against a 200 Hz IMU (gazebo/worlds/camera_world.sdf).
# One prediction per 5 ms block cuts the pinned predict() calls 5x (run
# 20261117-flightfea1c93152a2: frame processing exceeded the 100 ms frame period
# once airborne). An IMU at <= 200 Hz is propagated sample by sample, unchanged.
PROPAGATION_STEP_S = .005


def new_pending():
    """Per-frame accumulators: gyro samples (dt, gyro) and max IMU gap."""
    return dict(pending_gyro=[], pending_max_gap=.001, pending_count=0)


def preintegration_bound(frame_stamp, period, newest_arrived=None):
    """Latest IMU stamp that may be integrated without passing a frame that may
    still be processed.

    A frame waiting in the slot (newest_arrived > frame_stamp) will be processed
    or superseded by a newer one, so integration may run up to just below it;
    frames it superseded are never processed. With nothing waiting, the next
    frame cannot be earlier than frame_stamp + period.
    """
    if frame_stamp is None or period is None or not (math.isfinite(frame_stamp) and math.isfinite(period)) or period <= 0:
        return None
    if newest_arrived is not None and newest_arrived > frame_stamp:
        return newest_arrived-STAMP_EPSILON_S
    return frame_stamp+period-STAMP_EPSILON_S


def advance(state, samples, base, until, predict, rotation, limit=None, step_s=0.):
    """Integrate consecutive IMU samples with t <= until, starting at state['imu_index'].

    samples[i] has absolute buffer index base+i. Updates last_imu, filter_t,
    imu_index and the pending accumulators. Stops BEFORE a gap/reset/non-finite
    sample and returns it without consuming it, so the frame path fails closed
    exactly as before. Returns (count, bad); count is raw samples consumed.

    step_s == 0: one EKF prediction per IMU sample, exactly as the frame path
    always did. step_s > 0: consecutive samples spanning up to step_s are
    propagated as ONE prediction with the interval-weighted mean specific force
    (identical velocity increment in the body frame), the rotation of the
    sample held at the block midpoint, and the interval-weighted mean gyro
    (the per-frame gyro mean is unchanged). Every raw sample still passes the
    gap/non-finite check and feeds max_gap. See PROPAGATION_STEP_S.
    """
    import numpy as np
    count = 0
    while True:
        i = state['imu_index']-base
        if i < 0:
            raise ValueError('IMU index before buffer window')
        block, bad, previous = [], None, state['last_imu']
        while i+len(block) < len(samples) and (limit is None or count+len(block) < limit):
            m = samples[i+len(block)]
            if m['t'] > until:
                break
            gap = m['t']-previous['t']
            if not 0 < gap <= .02 or not all(map(math.isfinite, m['f']+m['gyro'])):
                bad = m
                break
            block.append((m, gap))
            previous = m
            if m['t']-state['filter_t'] >= step_s-1e-12:
                break
        if not block:
            return count, bad
        last, start = state['last_imu'], state['filter_t']
        if len(block) == 1:
            m, gap = block[0]
            dt = m['t']-start
            # rotation() may raise ValueError before anything is consumed.
            r = rotation(last['q']) if dt > 0 else None
            if dt > 0:
                predict(np.asarray(last['f']), r, dt)
                state['pending_gyro'].append((dt, np.asarray(last['gyro'])))
        else:
            holds, edges = [last]+[m for m, _ in block[:-1]], [start]+[m['t'] for m, _ in block]
            weights = [max(0., b-a) for a, b in zip(edges, edges[1:])]
            dt = sum(weights)
            if dt > 0:
                middle = start+dt/2
                hold = next(h for h, b in zip(holds, edges[1:]) if b >= middle)
                r = rotation(hold['q'])
                f, g = [0., 0., 0.], [0., 0., 0.]
                for w, h in zip(weights, holds):
                    hf, hg = h['f'], h['gyro']
                    for k in range(3):
                        f[k] += w*hf[k]
                        g[k] += w*hg[k]
                predict(np.array(f)/dt, r, dt)
                state['pending_gyro'].append((dt, np.array(g)/dt))
        state['pending_max_gap'] = max([state['pending_max_gap']]+[gap for _, gap in block])
        state.update(last_imu=block[-1][0], filter_t=block[-1][0]['t'], imu_index=state['imu_index']+len(block))
        state['pending_count'] += len(block)
        count += len(block)
        if bad is not None:
            return count, bad


class ImuGapError(ValueError):
    """A gap/reset/non-finite IMU sample: the caller starts a new estimator epoch."""


class PreintegrationOverrun(ValueError):
    """Pre-integration passed this frame's time (camera grid changed): new epoch."""


def frame_step(state, imu, t, predict, rotation, step_s=0.):
    """Frame-path IMU step: remaining samples <= t, then the partial step to t.

    Identical to the loop the frame path always ran; it now usually finds the
    samples already integrated by idle_step. Returns (gyro_samples, max_gap,
    last_sample, rotation_at_t, imu_age).
    """
    import numpy as np
    if state['filter_t'] > t:
        raise PreintegrationOverrun('IMU pre-integration passed frame time')
    _, bad = advance(state, imu, 0, t, predict, rotation, step_s=step_s)
    if bad is not None:
        raise ImuGapError('IMU gap/reset/nonfinite')
    gyro_samples, max_gap = state['pending_gyro'], state['pending_max_gap']
    state.update(new_pending())
    last = state['last_imu']
    imu_age = t-last['t']
    if not 0 <= imu_age <= .005:
        raise ValueError('IMU watermark/alignment')
    r = rotation(last['q'])
    dt = t-state['filter_t']
    if dt > 0:
        predict(np.asarray(last['f']), r, dt)
        gyro_samples.append((dt, np.asarray(last['gyro'])))
    state['filter_t'] = t
    return gyro_samples, max_gap, last, r, imu_age


def idle_step(state, tail, predict, rotation, newest_arrived=None, step_s=0.):
    """Between frames: integrate up to TICK_CHUNK samples, never past the next frame.

    tail(index, limit) returns buffer items from index. Does nothing while the
    estimator is uninitialized or awaiting a reset. Returns samples integrated.
    """
    bound = preintegration_bound(state['frame_stamp'], state['camera_period'], newest_arrived)
    if (bound is None or state['last_imu'] is None or state['reset_after'] is not None
            or state['filter_t'] is None or state['filter_t'] >= bound):
        return 0
    samples = tail(state['imu_index'], TICK_CHUNK)
    if not samples:
        return 0
    count, _ = advance(state, samples, state['imu_index'], bound, predict, rotation, limit=TICK_CHUNK, step_s=step_s)
    return count


def observe_frame_stamp(state, t):
    """Track the camera period from frame stamps; return frames never received.

    Frames between the previous and this stamp that were neither processed nor
    reported superseded by the slot accounting were never received.
    """
    previous = state['frame_stamp']
    missing = 0
    if previous is not None and t > previous:
        interval = t-previous
        state['camera_period'] = interval if state['camera_period'] is None else min(state['camera_period'], interval)
        missing = max(0, round(interval/state['camera_period'])-1-state['superseded_since_frame'])
    state['superseded_since_frame'] = 0
    state['frame_stamp'] = t
    return missing
