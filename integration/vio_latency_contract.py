"""Latency contract between the VIO source and the transport (simulation seconds).

The transport grants each sample a host-time budget of
    TRANSPORT_FRESHNESS - (sim clock - acquisition) - mapping uncertainty
(navigation_lifecycle.NavigationLifecycle.pair) and expires it when its own
pairing runs past that budget. Before this contract the source published
samples up to 180 ms old, while the transport could only use ones younger than
about 175 ms (200 - 17 ms uncertainty - ~8 ms pairing). A sample published at
175 ms (acquisition 37.9 s, run 20261117-flight4414cb818f2a) was therefore
certain to expire and latch the flight.

The source limit is derived from the transport's unchanged limits, so every
published sample still has budget left when the transport finishes pairing it.
This tightens the source gate; no transport gate is relaxed. The transport
logs (never acts on) any sample that exceeds a reserve, so the reserves can be
re-derived from the next run.
"""

# Transport side, unchanged, enforced there (tests pin these against the code).
TRANSPORT_FRESHNESS_S = .200        # navigation_lifecycle pair budget / stale-acquisition limit
MAPPING_UNCERTAINTY_MAX_S = .020    # physics_clock_mapping max_uncertainty_ns: larger is rejected

# Reserves measured from preserved flights (integration/debug_evidence):
# source publication check -> transport pair decision, max 11 ms (4414cb818f2a)
# and 15 ms (6f32c08e7aaf); transport pair + durable admit + egress approve,
# max 7.86 ms and 6.30 ms.
DELIVERY_RESERVE_S = .015
PAIR_PROCESSING_RESERVE_S = .008

SOURCE_PUBLICATION_MAX_AGE_S = round(TRANSPORT_FRESHNESS_S-MAPPING_UNCERTAINTY_MAX_S
                                     - DELIVERY_RESERVE_S-PAIR_PROCESSING_RESERVE_S, 6)   # 0.157


def describe():
    return dict(transport_freshness_s=TRANSPORT_FRESHNESS_S, mapping_uncertainty_max_s=MAPPING_UNCERTAINTY_MAX_S,
                delivery_reserve_s=DELIVERY_RESERVE_S, pair_processing_reserve_s=PAIR_PROCESSING_RESERVE_S,
                source_publication_max_age_s=SOURCE_PUBLICATION_MAX_AGE_S)
