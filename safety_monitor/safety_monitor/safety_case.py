"""Machine-readable safety argument (config/safety_case.yaml) and claim status.

Pure Python, no ROS imports: shared by safety_monitor_node (thresholds and
claim ids), analysis/analyze_runs.py (claim status, GSN export) and the tests.
"""
import yaml

SUPPORTED = 'supported'
CHALLENGED = 'challenged'
NO_EVIDENCE = 'no_evidence'
STATUSES = (SUPPORTED, CHALLENGED, NO_EVIDENCE)

EXPOSURE_KINDS = ('speed_zone', 'near_keepout', 'whole_run')
MIN_EXPOSURE_KEYS = ('time_s', 'distance_m')

# SPI-2/SPI-3 already publish the amount beyond the threshold as peak_value
# (penetration depth; threshold - clearance), SPI-1/SPI-4 the raw measurement.
PEAK_IS_EXCESS = frozenset({'SPI-2', 'SPI-3'})


class SafetyCaseError(ValueError):
    pass


class SafetyCase:

    def __init__(self, data):
        sc = data.get('safety_case', data)
        self.id = sc.get('id', '')
        self.title = sc.get('title', '')
        self.goals = dict(sc.get('goals') or {})
        self.evidence = dict(sc.get('evidence') or {})
        self._validate()

    @classmethod
    def from_file(cls, path):
        with open(path) as f:
            return cls(yaml.safe_load(f))

    def _validate(self):
        if not self.goals:
            raise SafetyCaseError('safety case defines no goals')
        for gid, goal in self.goals.items():
            for child in goal.get('supported_by', []) or []:
                if child not in self.goals:
                    raise SafetyCaseError(f'goal {gid}: unknown supporting goal {child}')
        for spi_id, ev in self.evidence.items():
            gid = ev.get('goal_id')
            if gid is not None and gid not in self.goals:
                raise SafetyCaseError(f'{spi_id}: unknown goal_id {gid}')
            if 'threshold' not in ev:
                raise SafetyCaseError(f'{spi_id}: missing threshold')
            kind = (ev.get('exposure') or {}).get('kind')
            if kind not in EXPOSURE_KINDS:
                raise SafetyCaseError(f'{spi_id}: exposure.kind must be one of {EXPOSURE_KINDS}')
            if kind == 'near_keepout' and 'within_m' not in ev['exposure']:
                raise SafetyCaseError(f'{spi_id}: near_keepout exposure needs within_m')
            for key in (ev.get('min_exposure') or {}):
                if key not in MIN_EXPOSURE_KEYS:
                    raise SafetyCaseError(f'{spi_id}: unknown min_exposure key {key}')

    # -- lookups ---------------------------------------------------------------
    def threshold(self, spi_id):
        return float(self.evidence[spi_id]['threshold'])

    def tolerance(self, spi_id):
        return float(self.evidence[spi_id].get('tolerance', 0.0))

    def claim_id(self, spi_id):
        """Goal id carried as SafetyEvent.claim_id ('' for informational SPIs)."""
        return self.evidence[spi_id].get('goal_id') or ''

    def evidence_for_goal(self, goal_id):
        return [s for s, ev in self.evidence.items() if ev.get('goal_id') == goal_id]

    def children(self, goal_id):
        return list(self.goals[goal_id].get('supported_by', []) or [])

    def goal_order(self):
        """Goals ordered so every goal comes after the goals supporting it."""
        order, seen = [], set()

        def visit(gid, stack):
            if gid in seen:
                return
            if gid in stack:
                raise SafetyCaseError(f'cycle in supported_by at {gid}')
            for child in self.children(gid):
                visit(child, stack | {gid})
            seen.add(gid)
            order.append(gid)

        for gid in self.goals:
            visit(gid, frozenset())
        return order

    # -- event filtering ---------------------------------------------------
    def counts_against(self, spi_id, peak_value):
        """True if an event with this peak exceeds threshold + tolerance."""
        threshold = self.threshold(spi_id)
        excess = peak_value if spi_id in PEAK_IS_EXCESS else peak_value - threshold
        return excess > self.tolerance(spi_id)


# -- claim status ------------------------------------------------------------

def meets_min_exposure(exposure, min_exposure):
    return all(exposure.get(key, 0.0) >= float(value)
               for key, value in (min_exposure or {}).items())


def evidence_status(event_count, exposure, min_exposure):
    """challenged: >= 1 event; supported: 0 events and enough exposure;
    no_evidence: 0 events but too little exposure to say anything."""
    if event_count > 0:
        return CHALLENGED
    if meets_min_exposure(exposure, min_exposure):
        return SUPPORTED
    return NO_EVIDENCE


def combine(statuses):
    """Any challenged -> challenged; all supported -> supported; else no_evidence."""
    statuses = list(statuses)
    if any(s == CHALLENGED for s in statuses):
        return CHALLENGED
    if statuses and all(s == SUPPORTED for s in statuses):
        return SUPPORTED
    return NO_EVIDENCE


def _ratio(num, den):
    return num / den if den else None


def assess_run(case, events_by_spi, exposure_by_spi):
    """Claim status for every goal of one run.

    events_by_spi:   spi_id -> list of {'peak_value', 'duration_s'} (all
                     events in the window; tolerance is applied here)
    exposure_by_spi: spi_id -> {'time_s', 'distance_m'}
    Returns one dict per goal, in safety-case order.
    """
    results = {}
    for gid in case.goal_order():
        evidence = []
        for spi_id in case.evidence_for_goal(gid):
            events = [e for e in events_by_spi.get(spi_id, [])
                      if case.counts_against(spi_id, e['peak_value'])]
            exposure = exposure_by_spi.get(spi_id, {})
            count = len(events)
            duration = sum(e.get('duration_s', 0.0) for e in events)
            evidence.append({
                'spi_id': spi_id,
                'status': evidence_status(
                    count, exposure, case.evidence[spi_id].get('min_exposure')),
                'event_count': count,
                'peak_value': max((e['peak_value'] for e in events), default=None),
                'event_duration_s': duration,
                'exposure_time_s': exposure.get('time_s', 0.0),
                'exposure_distance_m': exposure.get('distance_m', 0.0),
                'duration_per_exposure': _ratio(duration, exposure.get('time_s', 0.0)),
                'events_per_m': _ratio(count, exposure.get('distance_m', 0.0)),
            })
        child_statuses = [results[c]['status'] for c in case.children(gid)]
        status = combine([e['status'] for e in evidence] + child_statuses)

        row = {'goal_id': gid, 'status': status,
               'spi_ids': ' '.join(e['spi_id'] for e in evidence),
               'supported_by': ' '.join(case.children(gid))}
        if len(evidence) == 1:
            row.update({k: v for k, v in evidence[0].items() if k not in ('spi_id', 'status')})
        else:
            # Exposure-normalised metrics are only meaningful for a single SPI.
            row.update({
                'event_count': sum(e['event_count'] for e in evidence),
                'peak_value': None, 'event_duration_s': sum(e['event_duration_s'] for e in evidence),
                'exposure_time_s': None, 'exposure_distance_m': None,
                'duration_per_exposure': None, 'events_per_m': None,
            })
        results[gid] = row
    return [results[gid] for gid in case.goals]


def summarize(rows, group_keys):
    """Counts supported/challenged/no_evidence per group of claim-status rows."""
    groups = {}
    for row in rows:
        key = tuple(row[k] for k in group_keys)
        counts = groups.setdefault(key, {s: 0 for s in STATUSES})
        counts[row['status']] += 1
    out = []
    for key, counts in groups.items():
        entry = dict(zip(group_keys, key))
        entry['n_runs'] = sum(counts.values())
        entry.update(counts)
        out.append(entry)
    return out


# -- GSN export ----------------------------------------------------------------

_FILL = {CHALLENGED: '#f4c7c3', SUPPORTED: '#cfe8cf', 'mixed': '#fff2b3', NO_EVIDENCE: '#e6e6e6'}


def _overall(counts):
    if counts.get(CHALLENGED, 0):
        return CHALLENGED
    if counts.get(SUPPORTED, 0) and not counts.get(NO_EVIDENCE, 0):
        return SUPPORTED
    if counts.get(SUPPORTED, 0):
        return 'mixed'
    return NO_EVIDENCE


def _esc(text):
    return str(text).replace('\\', '\\\\').replace('"', '\\"')


def to_dot(case, goal_counts, title=''):
    """GSN fragment as Graphviz DOT; goals coloured by claim status.

    goal_counts: goal_id -> {'supported': n, 'challenged': n, 'no_evidence': n}
    Fill: red = >= 1 run challenged, green = all runs supported,
    yellow = some supported / rest no evidence, grey = no evidence at all.
    """
    lines = ['digraph GSN {', '  rankdir=TB;', '  node [fontname="Helvetica", fontsize=10];']
    if title:
        lines.append(f'  labelloc="t"; label="{_esc(title)}";')
    for gid, goal in case.goals.items():
        counts = goal_counts.get(gid, {})
        tally = ' / '.join(f'{s[0].upper()} {counts.get(s, 0)}' for s in STATUSES)
        label = (_esc(f'{gid}: {goal.get("text", "")}') + '\\n'
                 + _esc(f'[{goal.get("hazard_id", "")}]  {tally}'))
        fill = _FILL[_overall(counts)]
        lines.append(f'  "{gid}" [shape=box, style=filled, fillcolor="{fill}", label="{label}"];')
        for child in case.children(gid):
            lines.append(f'  "{gid}" -> "{child}";')
    for spi_id, ev in case.evidence.items():
        sid = f'Sn_{spi_id}'
        label = _esc(spi_id) + '\\n' + _esc(f'thr {ev["threshold"]} {ev.get("unit", "")}')
        lines.append(f'  "{sid}" [shape=ellipse, label="{label}"];')
        if ev.get('goal_id'):
            lines.append(f'  "{ev["goal_id"]}" -> "{sid}";')
    lines.append('}')
    return '\n'.join(lines) + '\n'
