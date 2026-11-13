"""Machine-readable safety argument (config/safety_case.yaml) and claim status.

Pure Python, no ROS imports: shared by safety_monitor_node (thresholds and
claim ids), analysis/analyze_runs.py (claim status), analysis/make_gsn.py and the tests.
"""
import textwrap

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
        self.context = dict(sc.get('context') or {})
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
FILL_LEGEND = ('fill: red = challenged in >= 1 run, green = supported in all runs, '
               'yellow = supported / rest no evidence, grey = no evidence')


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


def _label(*lines, width=24):
    """DOT label from lines, each escaped and wrapped at `width` characters."""
    out = []
    for line in lines:
        out += textwrap.wrap(str(line), width) or ['']
    return '\\n'.join(_esc(line) for line in out)


def _id_range(ids):
    ids = list(ids)
    return f'{ids[0]}–{ids[-1]}' if len(ids) > 2 else ', '.join(ids)


def to_dot(case, goal_counts=None, title='', tolerances=None):
    """GSN (Goal Structuring Notation) as Graphviz DOT.

    Goals are rectangles, strategies parallelograms, solutions circles and
    context rounded rectangles; SupportedBy edges have filled arrowheads,
    InContextOf edges hollow ones. Evidence without a goal (informational)
    is drawn as dashed context of the top goal.

    goal_counts: None for the structure only, or goal_id -> {'supported': n,
                 'challenged': n, 'no_evidence': n}: goals are then filled by
                 overall status (see FILL_LEGEND) and show the tally.
    tolerances:  spi_id -> tolerance to show instead of the safety case's own.
    """
    tolerances = tolerances or {}
    lines = ['digraph GSN {',
             '  rankdir=TB; nodesep=0.25; ranksep=0.35; margin=0;',
             '  node [fontname="Helvetica", fontsize=10, margin="0.08,0.04"];',
             '  edge [arrowsize=0.7];']
    if title:
        lines.append(f'  labelloc="t"; fontname="Helvetica"; fontsize=10; label="{_label(title, width=80)}";')

    children = {c for gid in case.goals for c in case.children(gid)}
    roots = [gid for gid in case.goals if gid not in children]
    strategy_of = {}
    for gid, goal in case.goals.items():
        counts = (goal_counts or {}).get(gid)
        text = [gid, goal.get('short') or goal.get('text', '')]
        style = ''
        if goal_counts is not None:
            counts = counts or {}
            text.append(' / '.join(f'{s[0].upper()} {counts.get(s, 0)}' for s in STATUSES))
            style = f', style=filled, fillcolor="{_FILL[_overall(counts)]}"'
        lines.append(f'  "{gid}" [shape=box{style}, label="{_label(*text)}"];')
        kids = case.children(gid)
        if kids:
            sid = f'S{len(strategy_of) + 1}'
            strategy_of[gid] = sid
            hazards = _id_range(case.goals[k].get('hazard_id', '') for k in kids)
            text = goal.get('strategy') or 'Argument over sub-goals'
            lines.append(f'  "{sid}" [shape=parallelogram, margin="0,0", '
                         f'label="{_label(sid, f"{text} {hazards}", width=18)}"];')
            lines.append(f'  "{gid}" -> "{sid}";')
            for kid in kids:
                lines.append(f'  "{sid}" -> "{kid}";')

    n = 0
    informational = []
    for spi_id, ev in case.evidence.items():
        if not ev.get('goal_id'):
            informational.append(spi_id)
            continue
        n += 1
        lines.append(f'  "Sn{n}" [shape=circle, fixedsize=false, label="{_label(f"Sn{n}", spi_id)}"];')
        lines.append(f'  "{ev["goal_id"]}" -> "Sn{n}";')

    # Context: the safety case's own entries, then thresholds/tolerances (C<n+1>).
    anchor = roots[0]
    contexts = list((case.context or {}).items())
    def with_unit(value, ev):
        return f'{value:g} {ev["unit"]}' if ev.get('unit') else f'{value:g}'

    thresholds = [f'{s}: {with_unit(float(ev["threshold"]), ev)} '
                  f'(tol. {with_unit(float(tolerances.get(s, ev.get("tolerance", 0.0))), ev)})'
                  for s, ev in case.evidence.items()]
    contexts.append((f'C{len(contexts) + 1}', None))
    # Layout: the case's own context left of the top goal, thresholds right of
    # it, informational SPIs right of the strategy -- keeps every row about as
    # wide as the sub-goal row.
    strategy = strategy_of.get(anchor, anchor)
    for cid, text in contexts:
        if text is None:
            label = '\\n'.join([_esc(cid), _esc('Threshold (tolerance)'), _esc('per SPI:')]
                               + [_esc(t) for t in thresholds])
            lines.append(f'  "{cid}" [shape=box, style=rounded, label="{label}"];')
            lines.append(f'  "{anchor}" -> "{cid}" [arrowhead=empty];')
        else:
            lines.append(f'  "{cid}" [shape=box, style=rounded, label="{_label(cid, text, width=22)}"];')
            # Drawn tail-first so the node lands left of the goal; the hollow
            # InContextOf head still points at the context.
            lines.append(f'  "{cid}" -> "{anchor}" [dir=back, arrowtail=empty];')
        lines.append(f'  {{ rank=same; "{anchor}"; "{cid}"; }}')
    for spi_id in informational:
        ev = case.evidence[spi_id]
        label = _label(f'{spi_id}: {ev.get("short", "")}', 'informational, supports no claim',
                       width=16)
        lines.append(f'  "{spi_id}" [shape=box, style="rounded,dashed", label="{label}"];')
        lines.append(f'  "{strategy}" -> "{spi_id}" [arrowhead=empty, style=dashed];')
        lines.append(f'  {{ rank=same; "{strategy}"; "{spi_id}"; }}')
    lines.append('}')
    return '\n'.join(lines) + '\n'
