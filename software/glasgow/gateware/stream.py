from amaranth.hdl import *
from amaranth._utils import final
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out


@final
class Signature(wiring.Signature):
    def __init__(self, payload_shape: 'ShapeLike', *, always_valid=False, always_ready=False):
        Shape.cast(payload_shape)
        self._payload_shape = payload_shape
        self._always_valid = bool(always_valid)
        self._always_ready = bool(always_ready)

        super().__init__({
            "payload": Out(payload_shape),
            "valid": Out(1),
            "ready": In(1)
        })

    # payload_shape intentionally not introspectable (for now)

    @property
    def always_valid(self):
        return self._always_valid

    @property
    def always_ready(self):
        return self._always_ready

    def __eq__(self, other):
        return (type(other) is type(self) and
            other._payload_shape == self._payload_shape and
            other.always_valid == self.always_valid and
            other.always_ready == self.always_ready)

    def create(self, *, path=None, src_loc_at=0):
        return Interface(self, path=path, src_loc_at=1 + src_loc_at)

    def __repr__(self):
        always_valid_repr = "" if not self._always_valid else ", always_valid=True"
        always_ready_repr = "" if not self._always_ready else ", always_ready=True"
        return f"stream.Signature({self._payload_shape!r}{always_valid_repr}{always_ready_repr})"


@final
class Interface:
    payload: Signal
    valid: 'Signal | Const'
    ready: 'Signal | Const'

    def __init__(self, signature: Signature, *, path=None, src_loc_at=0):
        if not isinstance(signature, Signature):
            raise TypeError(f"Signature of stream.Interface must be a stream.Signature, not "
                            f"{signature!r}")
        self._signature = signature
        self.__dict__.update(signature.members.create(path=path, src_loc_at=1 + src_loc_at))
        if signature.always_valid:
            self.valid = Const(1)
        if signature.always_ready:
            self.ready = Const(1)

    @property
    def signature(self):
        return self._signature

    @property
    def p(self):
        return self.payload

    def __repr__(self):
        return f"stream.Interface(payload={self.payload!r}, valid={self.valid!r}, ready={self.ready!r})"
