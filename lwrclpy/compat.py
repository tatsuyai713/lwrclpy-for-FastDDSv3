from types import ModuleType
import inspect


def _patch_message_class(cls):
    if getattr(cls, "__lwrclpy_attr_patched__", False):
        return
    if cls.__name__.endswith("PubSubType"):
        return
    try:
        inst = cls()
    except Exception:
        return

    simple_fields = []
    for name in dir(inst):
        if name.startswith("_"):
            continue
        try:
            attr = getattr(inst, name)
        except Exception:
            continue
        if not callable(attr):
            continue
        try:
            sig = inspect.signature(attr)
            params = list(sig.parameters.values())
            if any(p.default is inspect._empty and p.kind == inspect.Parameter.POSITIONAL_OR_KEYWORD for p in params[1:]):
                continue
        except Exception:
            pass
        try:
            attr()
        except TypeError:
            continue
        except Exception:
            continue
        simple_fields.append(name)

    if not simple_fields:
        return

    def __getattr__(self, name):
        if name in simple_fields:
            try:
                attr = object.__getattribute__(self, name)
                if callable(attr):
                    return attr()
            except Exception:
                pass
        raise AttributeError(name)

    def __setattr__(self, name, value):
        if name in simple_fields:
            try:
                attr = object.__getattribute__(self, name)
                if callable(attr):
                    try:
                        attr(value)
                        return
                    except Exception:
                        pass
            except Exception:
                pass
        object.__setattr__(self, name, value)

    # Preserve existing __getattr__/__setattr__ if present by chaining
    if not hasattr(cls, "__getattr__"):
        cls.__getattr__ = __getattr__
    if not hasattr(cls, "__setattr__"):
        cls.__setattr__ = __setattr__
    setattr(cls, "__lwrclpy_attr_patched__", True)


def _patch_module(mod: ModuleType):
    for name in dir(mod):
        try:
            obj = getattr(mod, name)
        except Exception:
            continue
        if isinstance(obj, type):
            _patch_message_class(obj)


def patch_known_message_modules():
    for mod_name in ("sensor_msgs.msg", "std_msgs.msg", "builtin_interfaces.msg"):
        try:
            module = __import__(mod_name, fromlist=["msg"])
        except Exception:
            continue
        _patch_module(module)


def patch_loaded_msg_modules():
    import sys

    for name, module in list(sys.modules.items()):
        if not module:
            continue
        if name.endswith(".msg") or ".msg." in name:
            _patch_module(module)


def ensure_pointfield_constants():
    """
    Backfill PointField constants expected by sensor_msgs_py (INT8/UINT8/INT16/...).
    Generated Fast DDS bindings may miss these enums; add them if absent.
    """
    try:
        from sensor_msgs.msg import PointField
    except Exception:
        return
    # Backfill constructor to accept keyword args (name/offset/datatype/count)
    if not getattr(PointField, "__lwrclpy_init_patched__", False):
        _orig_init = PointField.__init__

        from .message_utils import _ValueProxy  # local import to avoid cycles

        def _patched_init(self, **kwargs):
            try:
                _orig_init(self)
            except Exception:
                pass
            for k, v in kwargs.items():
                try:
                    setter = getattr(self, k, None)
                    if callable(setter):
                        try:
                            setter(v)
                            try:
                                object.__setattr__(self, k, _ValueProxy(v))
                            except Exception:
                                pass
                            continue
                        except Exception:
                            pass
                    setattr(self, k, v)
                    try:
                        object.__setattr__(self, k, _ValueProxy(v))
                    except Exception:
                        pass
                except Exception:
                    continue

        try:
            PointField.__init__ = _patched_init
            PointField.__lwrclpy_init_patched__ = True
        except Exception:
            pass

    constants = {
        "INT8": 1,
        "UINT8": 2,
        "INT16": 3,
        "UINT16": 4,
        "INT32": 5,
        "UINT32": 6,
        "FLOAT32": 7,
        "FLOAT64": 8,
    }
    for name, value in constants.items():
        if not hasattr(PointField, name):
            try:
                setattr(PointField, name, value)
            except Exception:
                pass
