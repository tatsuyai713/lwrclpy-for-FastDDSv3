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


def ensure_common_interface_constants():
    """
    Backfill well-known message constants that are present in ROS 2 rclpy-generated code
    but omitted in the SWIG/FastDDS bindings. Values are sourced from the IDL files in
    third_party/ros-data-types-for-fastdds.
    """
    mappings = [
        ("diagnostic_msgs.msg", "DiagnosticStatus", {
            "OK": 0, "WARN": 1, "ERROR": 2, "STALE": 3,
        }),
        ("sensor_msgs.msg", "BatteryState", {
            "POWER_SUPPLY_STATUS_UNKNOWN": 0,
            "POWER_SUPPLY_STATUS_CHARGING": 1,
            "POWER_SUPPLY_STATUS_DISCHARGING": 2,
            "POWER_SUPPLY_STATUS_NOT_CHARGING": 3,
            "POWER_SUPPLY_STATUS_FULL": 4,
            "POWER_SUPPLY_HEALTH_UNKNOWN": 0,
            "POWER_SUPPLY_HEALTH_GOOD": 1,
            "POWER_SUPPLY_HEALTH_OVERHEAT": 2,
            "POWER_SUPPLY_HEALTH_DEAD": 3,
            "POWER_SUPPLY_HEALTH_OVERVOLTAGE": 4,
            "POWER_SUPPLY_HEALTH_UNSPEC_FAILURE": 5,
            "POWER_SUPPLY_HEALTH_COLD": 6,
            "POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE": 7,
            "POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE": 8,
            "POWER_SUPPLY_TECHNOLOGY_UNKNOWN": 0,
            "POWER_SUPPLY_TECHNOLOGY_NIMH": 1,
            "POWER_SUPPLY_TECHNOLOGY_LION": 2,
            "POWER_SUPPLY_TECHNOLOGY_LIPO": 3,
            "POWER_SUPPLY_TECHNOLOGY_LIFE": 4,
            "POWER_SUPPLY_TECHNOLOGY_NICD": 5,
            "POWER_SUPPLY_TECHNOLOGY_LIMN": 6,
        }),
        ("sensor_msgs.msg", "NavSatStatus", {
            "STATUS_NO_FIX": 255,
            "STATUS_FIX": 0,
            "STATUS_SBAS_FIX": 1,
            "STATUS_GBAS_FIX": 2,
            "SERVICE_GPS": 1,
            "SERVICE_GLONASS": 2,
            "SERVICE_COMPASS": 4,
            "SERVICE_GALILEO": 8,
        }),
        ("sensor_msgs.msg", "Range", {
            "ULTRASOUND": 0,
            "INFRARED": 1,
        }),
        ("tf2_msgs.msg", "TF2Error", {
            "NO_ERROR": 0,
            "LOOKUP_ERROR": 1,
            "CONNECTIVITY_ERROR": 2,
            "EXTRAPOLATION_ERROR": 3,
            "INVALID_ARGUMENT_ERROR": 4,
            "TIMEOUT_ERROR": 5,
            "TRANSFORM_ERROR": 6,
        }),
        ("rcl_interfaces.msg", "ParameterValue", {
            "PARAMETER_NOT_SET": 0,
            "PARAMETER_BOOL": 1,
            "PARAMETER_INTEGER": 2,
            "PARAMETER_DOUBLE": 3,
            "PARAMETER_STRING": 4,
            "PARAMETER_BYTE_ARRAY": 5,
            "PARAMETER_BOOL_ARRAY": 6,
            "PARAMETER_INTEGER_ARRAY": 7,
            "PARAMETER_DOUBLE_ARRAY": 8,
            "PARAMETER_STRING_ARRAY": 9,
        }),
        ("visualization_msgs.msg", "InteractiveMarkerFeedback", {
            "KEEP_ALIVE": 0,
            "POSE_UPDATE": 1,
            "MENU_SELECT": 2,
            "BUTTON_CLICK": 3,
            "MOUSE_DOWN": 4,
            "MOUSE_UP": 5,
        }),
        ("sensor_msgs.msg", "JoyFeedback", {
            "TYPE_LED": 0,
            "TYPE_RUMBLE": 1,
            "TYPE_BUZZER": 2,
        }),
        ("visualization_msgs.msg", "MenuEntry", {
            "FEEDBACK": 0,
            "ROSRUN": 1,
            "ROSLAUNCH": 2,
        }),
        ("visualization_msgs.msg", "Marker", {
            "ARROW": 0,
            "CUBE": 1,
            "SPHERE": 2,
            "CYLINDER": 3,
            "LINE_STRIP": 4,
            "LINE_LIST": 5,
            "CUBE_LIST": 6,
            "SPHERE_LIST": 7,
            "POINTS": 8,
            "TEXT_VIEW_FACING": 9,
            "MESH_RESOURCE": 10,
            "TRIANGLE_LIST": 11,
            "ADD": 0,
            "MODIFY": 0,
            "DELETE": 2,
            "DELETEALL": 3,
        }),
        ("visualization_msgs.msg", "InteractiveMarkerControl", {
            "INHERIT": 0,
            "FIXED": 1,
            "VIEW_FACING": 2,
            "NONE": 0,
            "MENU": 1,
            "BUTTON": 2,
            "MOVE_AXIS": 3,
            "MOVE_PLANE": 4,
            "ROTATE_AXIS": 5,
            "MOVE_ROTATE": 6,
            "MOVE_3D": 7,
            "ROTATE_3D": 8,
            "MOVE_ROTATE_3D": 9,
        }),
        ("visualization_msgs.msg", "ImageMarker", {
            "CIRCLE": 0,
            "LINE_STRIP": 1,
            "LINE_LIST": 2,
            "POLYGON": 3,
            "POINTS": 4,
            "ADD": 0,
            "REMOVE": 1,
        }),
        ("visualization_msgs.msg", "InteractiveMarkerUpdate", {
            "KEEP_ALIVE": 0,
            "UPDATE": 1,
        }),
        ("action_msgs.srv", "CancelGoal_Request", {
            "ERROR_NONE": 0,
            "ERROR_REJECTED": 1,
            "ERROR_UNKNOWN": 2,
        }),
        ("action_msgs.msg", "GoalStatus", {
            "STATUS_UNKNOWN": 0,
            "STATUS_ACCEPTED": 1,
            "STATUS_EXECUTING": 2,
            "STATUS_CANCELING": 3,
            "STATUS_SUCCEEDED": 4,
            "STATUS_CANCELED": 5,
            "STATUS_ABORTED": 6,
        }),
        ("lifecycle_msgs.msg", "State", {
            "PRIMARY_STATE_UNKNOWN": 0,
            "PRIMARY_STATE_UNCONFIGURED": 1,
            "PRIMARY_STATE_INACTIVE": 2,
            "PRIMARY_STATE_ACTIVE": 3,
            "PRIMARY_STATE_FINALIZED": 4,
            "TRANSITION_STATE_CONFIGURING": 10,
            "TRANSITION_STATE_CLEANINGUP": 11,
            "TRANSITION_STATE_SHUTTINGDOWN": 12,
            "TRANSITION_STATE_ACTIVATING": 13,
            "TRANSITION_STATE_DEACTIVATING": 14,
            "TRANSITION_STATE_ERRORPROCESSING": 15,
        }),
        ("lifecycle_msgs.msg", "Transition", {
            "TRANSITION_CREATE": 0,
            "TRANSITION_CONFIGURE": 1,
            "TRANSITION_CLEANUP": 2,
            "TRANSITION_ACTIVATE": 3,
            "TRANSITION_DEACTIVATE": 4,
            "TRANSITION_SHUTDOWN": 5,
            "TRANSITION_DESTROY": 6,
            "TRANSITION_ON_CONFIGURE_SUCCESS": 10,
            "TRANSITION_ON_CONFIGURE_FAILURE": 11,
            "TRANSITION_ON_CONFIGURE_ERROR": 12,
            "TRANSITION_ON_CLEANUP_SUCCESS": 20,
            "TRANSITION_ON_CLEANUP_FAILURE": 21,
            "TRANSITION_ON_CLEANUP_ERROR": 22,
            "TRANSITION_ON_ACTIVATE_SUCCESS": 30,
            "TRANSITION_ON_ACTIVATE_FAILURE": 31,
            "TRANSITION_ON_ACTIVATE_ERROR": 32,
            "TRANSITION_ON_DEACTIVATE_SUCCESS": 40,
            "TRANSITION_ON_DEACTIVATE_FAILURE": 41,
            "TRANSITION_ON_DEACTIVATE_ERROR": 42,
            "TRANSITION_UNCONFIGURED_SHUTDOWN": 50,
            "TRANSITION_INACTIVE_SHUTDOWN": 51,
            "TRANSITION_ACTIVE_SHUTDOWN": 52,
            "TRANSITION_ON_SHUTDOWN_SUCCESS": 53,
            "TRANSITION_ON_SHUTDOWN_FAILURE": 54,
            "TRANSITION_ON_SHUTDOWN_ERROR": 55,
            "TRANSITION_ON_ERROR_SUCCESS": 60,
            "TRANSITION_ON_ERROR_FAILURE": 61,
            "TRANSITION_ON_ERROR_ERROR": 62,
            "TRANSITION_CALLBACK_SUCCESS": 97,
            "TRANSITION_CALLBACK_FAILURE": 98,
            "TRANSITION_CALLBACK_ERROR": 99,
        }),
        ("shape_msgs.msg", "SolidPrimitive", {
            "BOX": 1,
            "SPHERE": 2,
            "CYLINDER": 3,
            "CONE": 4,
            "BOX_X": 0,
            "BOX_Y": 1,
            "BOX_Z": 2,
            "SPHERE_RADIUS": 0,
            "CYLINDER_HEIGHT": 0,
            "CYLINDER_RADIUS": 1,
            "CONE_HEIGHT": 0,
            "CONE_RADIUS": 1,
        }),
        ("sensor_msgs.msg", "PointField", {
            "INT8": 1,
            "UINT8": 2,
            "INT16": 3,
            "UINT16": 4,
            "INT32": 5,
            "UINT32": 6,
            "FLOAT32": 7,
            "FLOAT64": 8,
        }),
        ("sensor_msgs.msg", "NavSatFix", {
            "COVARIANCE_TYPE_UNKNOWN": 0,
            "COVARIANCE_TYPE_APPROXIMATED": 1,
            "COVARIANCE_TYPE_DIAGONAL_KNOWN": 2,
            "COVARIANCE_TYPE_KNOWN": 3,
        }),
    ]

    for mod_name, cls_name, consts in mappings:
        try:
            mod = __import__(mod_name, fromlist=[cls_name])
            cls = getattr(mod, cls_name, None)
        except Exception:
            continue
        if not cls or not isinstance(cls, type):
            continue
        for name, value in consts.items():
            if hasattr(cls, name):
                continue
            try:
                setattr(cls, name, value)
            except Exception:
                continue


def _patch_kwargs_init(cls):
    """Allow msg classes to accept kwargs and assign via setters/attributes."""
    if getattr(cls, "__lwrclpy_kwargs_patched__", False):
        return
    _orig_init = getattr(cls, "__init__", None)
    if not callable(_orig_init):
        return

    def _patched_init(self, *args, **kwargs):
        try:
            _orig_init(self, *args)
        except Exception:
            try:
                _orig_init(self)
            except Exception:
                pass
        from .message_utils import _ValueProxy  # local import to avoid cycles
        for k, v in kwargs.items():
            try:
                attr = getattr(self, k, None)
                if callable(attr):
                    try:
                        attr(v)
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
        cls.__init__ = _patched_init
        cls.__lwrclpy_kwargs_patched__ = True
    except Exception:
        pass


def patch_kwargs_for_common_interfaces():
    """Patch common interface message classes to accept kwargs (ROS 2 style)."""
    modules = [
        "std_msgs.msg",
        "builtin_interfaces.msg",
        "sensor_msgs.msg",
        "geometry_msgs.msg",
        "nav_msgs.msg",
        "trajectory_msgs.msg",
        "diagnostic_msgs.msg",
        "shape_msgs.msg",
        "stereo_msgs.msg",
        "lifecycle_msgs.msg",
        "visualization_msgs.msg",
        "tf2_msgs.msg",
    ]
    for mod_name in modules:
        try:
            mod = __import__(mod_name, fromlist=["msg"])
        except Exception:
            continue
        for name in dir(mod):
            try:
                obj = getattr(mod, name)
            except Exception:
                continue
            if isinstance(obj, type) and not obj.__name__.endswith("PubSubType"):
                _patch_kwargs_init(obj)
