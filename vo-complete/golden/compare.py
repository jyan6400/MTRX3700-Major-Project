"""Record-by-record comparison of golden model, RTL replay and board capture."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from vlog import REC_FIELDS, Records

TYPE_NAMES = {
    "P": "adapter pixels + masks",
    "S": "adapter frame_start",
    "E": "adapter frame_end/frame_good",
    "D": "adapter diagnostics",
    "Z": "adapter final diagnostics",
    "R": "led_centroid results (robot path)",
    "T": "tri_led_detect results",
}


@dataclass
class TypeReport:
    rtype: str
    n_a: int
    n_b: int
    n_compared: int
    n_mismatch: int
    first: list = field(default_factory=list)       # (index, field, a, b)

    @property
    def ok(self):
        return self.n_mismatch == 0 and self.n_a == self.n_b


@dataclass
class Report:
    a_name: str
    b_name: str
    types: dict

    @property
    def ok(self):
        return all(t.ok for t in self.types.values())

    def summary(self):
        lines = [f"{self.a_name} vs {self.b_name}: {'MATCH' if self.ok else 'MISMATCH'}"]
        for k, t in self.types.items():
            lines.append(f"  {k} {TYPE_NAMES[k]:<34} {t.n_a:>8} vs {t.n_b:<8} "
                         f"{'ok' if t.ok else f'{t.n_mismatch} mismatching records'}")
            for i, f, va, vb in t.first[:5]:
                lines.append(f"      record {i}: {f} = {va} ({self.a_name}) vs {vb} ({self.b_name})")
        return "\n".join(lines)

    def table(self):
        import pandas as pd
        return pd.DataFrame([dict(type=k, what=TYPE_NAMES[k], **{self.a_name: t.n_a, self.b_name: t.n_b},
                                  mismatches=t.n_mismatch + abs(t.n_a - t.n_b), result="ok" if t.ok else "MISMATCH")
                             for k, t in self.types.items()])


def compare(a: Records, b: Records, a_name="model", b_name="RTL", types="PSEDZRT",
            skip_fields=None, max_first=20) -> Report:
    """Exact comparison, record by record in order, of every field of every
    record type. Fields where either side holds -1 (not observed) are skipped.
    skip_fields: {type: [field names]} not compared (e.g. Z cycle)."""
    skip_fields = skip_fields or {"Z": ["cycle"]}
    out = {}
    for k in types:
        A, B = a[k], b[k]
        names = REC_FIELDS[k]
        n = min(len(A), len(B))
        cols = [i for i, f in enumerate(names) if f not in skip_fields.get(k, [])]
        first = []
        bad_rows = 0
        if n:
            Aa, Bb = A[:n][:, cols], B[:n][:, cols]
            observed = (Aa != -1) & (Bb != -1)
            diff = (Aa != Bb) & observed
            rows = np.flatnonzero(diff.any(axis=1))
            bad_rows = len(rows)
            for r in rows[:max_first]:
                for j in np.flatnonzero(diff[r]):
                    first.append((int(r), names[cols[j]], int(Aa[r, j]), int(Bb[r, j])))
        if len(A) != len(B):
            first.append((n, "<record count>", len(A), len(B)))
        out[k] = TypeReport(k, len(A), len(B), n, bad_rows, first)
    return Report(a_name, b_name, out)
