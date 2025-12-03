import inspect
from types import ModuleType


def _patch_message_class(cls):
    if getattr(cls, "__lwrclpy_attr_patched__", False):
        return
    # Avoid patching PubSubType or non-SWIG classes
    if cls.__name__.endswith("PubSubType"):
        return
    try:
        inst = cls()
    except Exception:
        return

    def make_property(method_name, method_obj):
        def getter(self):
            try:
                return method_obj(self)
            except Exception:
                return None

        def setter(self, value):
            try:
                method_obj(self, value)
            except Exception:
                try:
                    object.__setattr__(self, method_name, value)
                except Exception:
                    pass

        return property(getter, setter)

    patched_any = False
    for name in dir(inst):
        if name.startswith("_"):
            continue
        try:
            attr = getattr(inst, name)
        except Exception:
            continue
        if not callable(attr):
            continue
        # Heuristic: only patch simple getters/setters (no required params)
        try:
            sig = inspect.signature(attr)
            params = sig.parameters
            # Allow bound methods with no required args
            if any(p.default is inspect._empty and p.kind == inspect.Parameter.POSITIONAL_OR_KEYWORD for p in list(params.values())[1:]):
                continue
        except Exception:
            # If signature is not introspectable, try calling without args
            pass
        try:
            attr()
        except TypeError:
            continue
        except Exception:
            continue

        # Replace method with property that delegates to the original method.
        setattr(cls, name, make_property(name, getattr(cls, name)))
        patched_any = True

    if patched_any:
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

