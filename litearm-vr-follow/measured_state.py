"""Read-only measured feedback protocol; never use command targets as feedback."""
import math
import time


def response(request_id, q=None, age_s=None, valid=False, error=None):
    try:
        q = [float(x) for x in q]
        age_s = float(age_s)
        valid = (valid and len(q) == 7 and all(math.isfinite(x) for x in q)
                 and math.isfinite(age_s) and 0 <= age_s <= 0.25)
    except (ValueError, TypeError):
        valid = False
    return {"ok": True, "cmd": "state", "request_id": request_id,
            "valid": bool(valid), "q": q if valid else None,
            "age_s": age_s if valid else None, "error": error}


class SequenceFreshness:
    """Require observed progress before trusting an untimestamped status cache.

    The first cache read cannot prove freshness. A changing firmware sequence
    proves a live stream; an unchanged sequence expires after 250 ms. A long
    polling gap also requires another transition, avoiding a stale cache being
    mistaken for a recent sample just because it differs from an ancient one.
    """
    def __init__(self):
        self.seq = None
        self.changed_at = None
        self.polled_at = None

    def age(self, seq, now=None):
        now = time.monotonic() if now is None else now
        if self.polled_at is None or now - self.polled_at > 0.25:
            self.seq, self.changed_at = seq, None
        elif seq != self.seq:
            self.seq, self.changed_at = seq, self.polled_at
        self.polled_at = now
        return None if self.changed_at is None else now - self.changed_at


def sdk_response(arm, request_id):
    try:
        st = arm.get_state()
        joints = st.get("feedback", {}).get("joints", [])
        valid = (len(joints) == 7 and all(j.get("fresh") is True for j in joints)
                 and not st.get("fault"))
        ages = [j.get("age_s") for j in joints]
        age = max(ages) if valid else None
        return response(request_id, st.get("q"), age, valid,
                        None if valid else "missing, stale or faulted measured feedback")
    except Exception as exc:
        return response(request_id, error=str(exc))
