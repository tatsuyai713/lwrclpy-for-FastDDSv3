# lwrclpy/message_utils.py
# Helpers for copying Fast-DDS-generated Python messages.
# Keeps compatibility with fastddsgen getter/setter style while honoring
# rclpy-like attribute assignment (msg.field = value).

from __future__ import annotations
import copy

# Fields injected by SWIG that should never be copied onto new instances.
_SKIP_FIELDS = {"this", "thisown"}


class _ValueProxy:
    """Callable wrapper so attribute access works like rclpy while keeping msg.field() usable."""

    __slots__ = ("_v",)

    def __init__(self, val):
        self._v = val

    def __call__(self):
        return self._v

    def __getattr__(self, name):
        return getattr(self._v, name)

    def __repr__(self):
        return repr(self._v)

    def __str__(self):
        return str(self._v)

    def __format__(self, spec):
        return format(self._v, spec)

    def __bytes__(self):
        return bytes(self._v) if hasattr(self._v, "__bytes__") else bytes(str(self._v), "utf-8")

    def __len__(self):
        return len(self._v) if hasattr(self._v, "__len__") else 0

    def __iter__(self):
        if hasattr(self._v, "__iter__"):
            return iter(self._v)
        raise TypeError(f"'{type(self._v).__name__}' object is not iterable")

    def __bool__(self):
        return bool(self._v)

    def __eq__(self, other):
        return self._v == other

    def __add__(self, other):
        try:
            return self._v + other
        except Exception:
            return NotImplemented

    def __radd__(self, other):
        try:
            return other + self._v
        except Exception:
            return NotImplemented


def expose_callable_fields(msg):
    """
    For a SWIG-generated message instance, expose callable zero-arg fields as
    attributes containing their current value (wrapped in _ValueProxy). This
    improves repr/debugging without cloning the whole message.
    """
    for name in dir(msg):
        if name.startswith("_"):
            continue
        try:
            attr = getattr(msg, name)
        except Exception:
            continue
        if not callable(attr):
            continue
        try:
            val = attr()
        except TypeError:
            continue
        except Exception:
            continue
        try:
            setattr(msg, name, _ValueProxy(val))
        except Exception:
            try:
                object.__setattr__(msg, name, _ValueProxy(val))
            except Exception:
                pass
    return msg


def clone_message(msg, msg_ctor):
    """
    Copy the received/sent message into a fresh instance using
    fastddsgen conventions:
      - getter/setter share the same name (foo() / foo(value))
      - getter fallback: get_foo()
    Also propagates plain attributes to support rclpy-style `msg.foo = x`.
    """

    def _copy_val(val):
        if isinstance(val, (str, int, float, bool, type(None))):
            return val
        if isinstance(val, (bytes, bytearray, memoryview)):
            try:
                return bytes(val)
            except Exception:
                pass
        # Buffer protocol fallback
        try:
            return bytes(val)
        except Exception:
            pass
        if isinstance(val, list):
            return [_copy_val(v) for v in val]
        if isinstance(val, tuple):
            return tuple(_copy_val(v) for v in val)
        if isinstance(val, set):
            return {_copy_val(v) for v in val}
        if isinstance(val, dict):
            return {k: _copy_val(v) for k, v in val.items()}
        if callable(val):
            try:
                return _copy_val(val())
            except Exception:
                return val
        try:
            return copy.deepcopy(val)
        except Exception:
            return val

    def _get_value(src, name):
        if name in _SKIP_FIELDS or name.startswith("_"):
            return None
        try:
            v = getattr(src, name)
        except Exception:
            return None
        # Instance attribute wins (rclpy-style msg.foo = val)
        if not callable(v):
            return v
        # Prefer zero-arg getter
        try:
            return v()
        except TypeError:
            pass
        except Exception:
            return None
        getter = f"get_{name}"
        try:
            gv = getattr(src, getter)
            if callable(gv):
                return gv()
        except Exception:
            pass
        return None

    def _assign(target, name, val) -> bool:
        if name in _SKIP_FIELDS or name.startswith("_"):
            return False
        try:
            setter = getattr(target, name, None)
            if callable(setter):
                try:
                    setter(val)
                except Exception:
                    pass
                # Expose rclpy-style attribute access while keeping callable behavior.
                try:
                    setattr(target, name, _ValueProxy(val))
                except Exception:
                    try:
                        object.__setattr__(target, name, _ValueProxy(val))
                    except Exception:
                        pass
                return True
        except Exception:
            pass
        # Fallback: explicit special-case for common fields present in __dict__ only
        if name == "data" and isinstance(val, (bytes, bytearray, memoryview)):
            try:
                target.data(val)
                return True
            except Exception:
                pass
        try:
            setattr(target, name, val)
            return True
        except Exception:
            try:
                object.__setattr__(target, name, val)
                return True
            except Exception:
                return False

    clone = msg_ctor()

    field_names = []
    # Values explicitly set on the instance (rclpy-like attribute assignment)
    if hasattr(msg, "__dict__"):
        field_names.extend(n for n in msg.__dict__ if not n.startswith("_"))
    field_names.extend(n for n in dir(msg) if not n.startswith("_"))
    field_names.extend(n for n in dir(clone) if not n.startswith("_"))

    seen = set()
    for name in field_names:
        if name in seen:
            continue
        seen.add(name)
        val = _get_value(msg, name)
        if val is None:
            continue
        copied = _copy_val(val)
        _assign(clone, name, copied)
    return clone
