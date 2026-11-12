"""Claim-status logic (challenged / supported / no_evidence) on synthetic data."""
import pytest

from safety_monitor.safety_case import (
    CHALLENGED, NO_EVIDENCE, SUPPORTED, SafetyCase, SafetyCaseError,
    assess_run, combine, evidence_status, summarize, to_dot)


def make_case(spi1_tolerance=0.0):
    return SafetyCase({'safety_case': {
        'id': 'SC-TEST',
        'goals': {
            'G1': {'text': 'top', 'hazard_id': 'H0', 'supported_by': ['G2', 'G3']},
            'G2': {'text': 'speed', 'hazard_id': 'H1'},
            'G3': {'text': 'keepout', 'hazard_id': 'H2'},
        },
        'evidence': {
            'SPI-1': {'goal_id': 'G2', 'threshold': 0.2, 'tolerance': spi1_tolerance,
                      'exposure': {'kind': 'speed_zone'},
                      'min_exposure': {'distance_m': 0.5, 'time_s': 1.0}},
            'SPI-2': {'goal_id': 'G3', 'threshold': 0.0,
                      'exposure': {'kind': 'near_keepout', 'within_m': 1.0},
                      'min_exposure': {'distance_m': 0.5}},
            'SPI-4': {'goal_id': None, 'threshold': 0.02,
                      'exposure': {'kind': 'whole_run'}, 'min_exposure': {}},
        },
    }})


def by_goal(rows):
    return {row['goal_id']: row for row in rows}


ENOUGH = {'SPI-1': {'time_s': 4.0, 'distance_m': 1.2}, 'SPI-2': {'time_s': 3.0, 'distance_m': 2.0}}


# -- single-evidence status ------------------------------------------------

def test_event_means_challenged_even_without_exposure():
    assert evidence_status(1, {'time_s': 0.0, 'distance_m': 0.0}, {'distance_m': 0.5}) == CHALLENGED


def test_no_events_and_enough_exposure_is_supported():
    assert evidence_status(0, {'time_s': 2.0, 'distance_m': 0.6}, {'distance_m': 0.5}) == SUPPORTED


def test_exposure_exactly_at_threshold_is_supported():
    assert evidence_status(0, {'distance_m': 0.5}, {'distance_m': 0.5}) == SUPPORTED


def test_no_events_and_too_little_exposure_is_no_evidence():
    assert evidence_status(0, {'time_s': 9.0, 'distance_m': 0.49}, {'distance_m': 0.5}) == NO_EVIDENCE


def test_every_min_exposure_key_must_be_met():
    min_exp = {'distance_m': 0.5, 'time_s': 1.0}
    assert evidence_status(0, {'distance_m': 2.0, 'time_s': 0.5}, min_exp) == NO_EVIDENCE
    assert evidence_status(0, {'distance_m': 2.0, 'time_s': 1.0}, min_exp) == SUPPORTED


def test_empty_min_exposure_is_always_met():
    assert evidence_status(0, {}, {}) == SUPPORTED


# -- aggregation -----------------------------------------------------------------

def test_combine_rules():
    assert combine([SUPPORTED, CHALLENGED, NO_EVIDENCE]) == CHALLENGED
    assert combine([SUPPORTED, SUPPORTED]) == SUPPORTED
    assert combine([SUPPORTED, NO_EVIDENCE]) == NO_EVIDENCE
    assert combine([]) == NO_EVIDENCE


def test_assess_run_all_supported():
    rows = by_goal(assess_run(make_case(), {}, ENOUGH))
    assert rows['G2']['status'] == SUPPORTED
    assert rows['G3']['status'] == SUPPORTED
    assert rows['G1']['status'] == SUPPORTED


def test_parent_goal_inherits_no_evidence():
    # e.g. a goal1 run: plenty of speed-zone exposure, never near keepout.
    exposure = {'SPI-1': ENOUGH['SPI-1'], 'SPI-2': {'time_s': 0.0, 'distance_m': 0.0}}
    rows = by_goal(assess_run(make_case(), {}, exposure))
    assert rows['G2']['status'] == SUPPORTED
    assert rows['G3']['status'] == NO_EVIDENCE
    assert rows['G1']['status'] == NO_EVIDENCE


def test_challenged_metrics_and_propagation():
    events = {'SPI-1': [{'peak_value': 0.46, 'duration_s': 1.5},
                        {'peak_value': 0.21, 'duration_s': 0.5}]}
    rows = by_goal(assess_run(make_case(), events, ENOUGH))
    g2 = rows['G2']
    assert g2['status'] == CHALLENGED
    assert g2['event_count'] == 2
    assert g2['peak_value'] == 0.46
    assert g2['event_duration_s'] == pytest.approx(2.0)
    assert g2['duration_per_exposure'] == pytest.approx(2.0 / 4.0)
    assert g2['events_per_m'] == pytest.approx(2 / 1.2)
    assert rows['G1']['status'] == CHALLENGED


def test_ratios_are_none_without_exposure():
    events = {'SPI-2': [{'peak_value': 0.05, 'duration_s': 0.0}]}
    exposure = {'SPI-1': ENOUGH['SPI-1'], 'SPI-2': {'time_s': 0.0, 'distance_m': 0.0}}
    g3 = by_goal(assess_run(make_case(), events, exposure))['G3']
    assert g3['status'] == CHALLENGED
    assert g3['duration_per_exposure'] is None
    assert g3['events_per_m'] is None


def test_tolerance_filters_small_exceedances():
    events = {'SPI-1': [{'peak_value': 0.2001, 'duration_s': 0.1}]}
    strict = by_goal(assess_run(make_case(spi1_tolerance=0.0), events, ENOUGH))['G2']
    tolerant = by_goal(assess_run(make_case(spi1_tolerance=0.005), events, ENOUGH))['G2']
    assert strict['status'] == CHALLENGED
    assert tolerant['status'] == SUPPORTED
    assert tolerant['event_count'] == 0


def test_informational_spi_never_affects_goals():
    events = {'SPI-4': [{'peak_value': 0.5, 'duration_s': 3.0}]}
    rows = by_goal(assess_run(make_case(), events, ENOUGH))
    assert all(row['status'] == SUPPORTED for row in rows.values())


def test_summarize_counts_per_group():
    rows = [{'config_id': 'a', 'goal_id': 'G2', 'status': SUPPORTED},
            {'config_id': 'a', 'goal_id': 'G2', 'status': CHALLENGED},
            {'config_id': 'a', 'goal_id': 'G2', 'status': SUPPORTED}]
    (entry,) = summarize(rows, ('config_id', 'goal_id'))
    assert entry['n_runs'] == 3
    assert entry[SUPPORTED] == 2
    assert entry[CHALLENGED] == 1
    assert entry[NO_EVIDENCE] == 0


# -- validation and export ---------------------------------------------------

def test_unknown_goal_reference_is_rejected():
    with pytest.raises(SafetyCaseError):
        SafetyCase({'goals': {'G1': {}},
                    'evidence': {'SPI-1': {'goal_id': 'G9', 'threshold': 0.2,
                                           'exposure': {'kind': 'whole_run'}}}})


def test_near_keepout_requires_distance():
    with pytest.raises(SafetyCaseError):
        SafetyCase({'goals': {'G1': {}},
                    'evidence': {'SPI-2': {'goal_id': 'G1', 'threshold': 0.0,
                                           'exposure': {'kind': 'near_keepout'}}}})


def test_shipped_safety_case_is_valid():
    from pathlib import Path
    case = SafetyCase.from_file(Path(__file__).parent.parent / 'config' / 'safety_case.yaml')
    assert case.claim_id('SPI-1') == 'G2'
    assert case.claim_id('SPI-4') == ''
    assert case.goal_order()[-1] == 'G1'


def test_dot_export_lists_goals_and_evidence():
    dot = to_dot(make_case(), {'G2': {CHALLENGED: 1, SUPPORTED: 2, NO_EVIDENCE: 0}})
    assert dot.startswith('digraph GSN')
    assert '"G1" -> "G2"' in dot
    assert '"G2" -> "Sn_SPI-1"' in dot
    assert '#f4c7c3' in dot  # G2 challenged in >= 1 run
