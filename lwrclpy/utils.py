"""ユーティリティ（生成型の解決をモジュール/クラス両対応に）。"""
import importlib
import types


TOPIC_PREFIX = "rt/"
SERVICE_REQUEST_PREFIX = "rq/"
SERVICE_RESPONSE_PREFIX = "rr/"
ACTION_PREFIX = "ra/"


def _normalize_namespace(ns: str) -> str:
    """Normalize namespace per ROS 2 rules (leading slash, no trailing slash except root)."""
    if not ns:
        return ""
    ns = "/" + ns.lstrip("/")
    if len(ns) > 1 and ns.endswith("/"):
        ns = ns.rstrip("/")
    return ns


def _join_with_namespace(ns: str, name: str) -> str:
    ns = ns.rstrip("/")
    name = name.lstrip("/")
    if not ns:
        return "/" + name if name else "/"
    if not name:
        return ns
    return ns + "/" + name


def resolve_name(name: str, namespace: str, node_name: str) -> str:
    """
    Resolve ROS graph names (topics/services) following ROS 2 name resolution rules:
      - absolute: /foo stays /foo
      - relative: foo -> <namespace>/foo
      - private: ~foo -> <namespace>/<node_name>/foo
    Reference: https://design.ros2.org/articles/topic_and_service_names.html
    Returns an absolute name starting with "/".
    """
    if not name:
        return _normalize_namespace(namespace) or "/"
    if name.startswith("~"):
        ns = _normalize_namespace(namespace)
        base = _join_with_namespace(ns or "/", node_name)
        return _join_with_namespace(base, name[1:])
    if name.startswith("/"):
        cleaned = name.lstrip("/")
        return "/" + cleaned
    ns = _normalize_namespace(namespace)
    return _join_with_namespace(ns or "/", name)


def resolve_generated_type(obj):
    """
    fastddsgen -python 生成物を指す `obj` (モジュール or クラス) から
    (module, msg_cls, pubsub_cls) を返す。

    期待する命名:
      - モジュールに <Name> クラスと <Name>PubSubType が同居
      - もしくはクラスを直接渡す（この場合は pubsub 名は <ClassName>PubSubType）
    """
    # 1) 基底モジュールを取得
    if isinstance(obj, type):  # クラスが渡された
        msg_cls = obj
        mod = importlib.import_module(obj.__module__)
        pubsub_name = obj.__name__ + "PubSubType"
        pubsub_cls = getattr(mod, pubsub_name, None)
        if pubsub_cls is None:
            # 安全側：モジュール内を総当たり
            pubsub_cls = _find_first_pubsub(mod, prefer=obj.__name__)
        if pubsub_cls is None:
            raise RuntimeError(
                f"PubSubType not found (expected '{pubsub_name}') in module '{mod.__name__}'"
            )
        return mod, msg_cls, pubsub_cls

    # モジュールが渡された場合
    if isinstance(obj, types.ModuleType):
        mod = obj
    else:
        # __init__.py の動的再エクスポートで来る可能性に備え、__module__ 経由で辿る
        mod = importlib.import_module(obj.__module__)

    # 2) モジュール内から候補抽出
    pubsub_cls, msg_cls = _pair_from_module(mod)
    if pubsub_cls is None or msg_cls is None:
        raise RuntimeError("Failed to resolve generated type (module/class mismatch)")
    return mod, msg_cls, pubsub_cls


def resolve_service_type(obj):
    """
    推定ルール:
      - rclpy 互換: <Srv>.Request / <Srv>.Response を持つ
      - fastddsgen 出力: <Srv>_request / <Srv>_response などがモジュールに居る
    戻り値: (request_cls, response_cls, request_pubsub_cls, response_pubsub_cls)
    """
    mod = None
    # rclpy スタイルのクラス (Request/Response 属性)
    if isinstance(obj, type):
        mod = importlib.import_module(obj.__module__)
        req = getattr(obj, "Request", None)
        res = getattr(obj, "Response", None)
    else:
        mod = importlib.import_module(obj.__module__)
        req = getattr(obj, "Request", None)
        res = getattr(obj, "Response", None)
    # fastddsgen 名寄せ (Foo_Request / Foo_Response)
    if req is None:
        req = getattr(mod, f"{obj.__name__}Request", None) if isinstance(obj, type) else None
    if res is None:
        res = getattr(mod, f"{obj.__name__}Response", None) if isinstance(obj, type) else None

    # fallback: scan module for *Request/*Response
    if req is None or res is None:
        for name in dir(mod):
            if req is None and name.lower().endswith("request"):
                cand = getattr(mod, name)
                if isinstance(cand, type):
                    req = cand
            if res is None and name.lower().endswith("response"):
                cand = getattr(mod, name)
                if isinstance(cand, type):
                    res = cand
    if req is None or res is None:
        raise RuntimeError("Could not resolve service Request/Response classes")

    _mod, req_cls, req_pubsub = resolve_generated_type(req)
    _mod, res_cls, res_pubsub = resolve_generated_type(res)
    return req_cls, res_cls, req_pubsub, res_pubsub


def resolve_action_type(action_type):
    """
    Resolve ROS2-style action generated classes.
    Expects attributes: Goal, Result, Feedback,
      SendGoal (with Request/Response),
      GetResult (with Request/Response),
      FeedbackMessage,
      CancelGoal (action_msgs/srv/CancelGoal)
    """
    goal_cls = getattr(action_type, "Goal", None)
    result_cls = getattr(action_type, "Result", None)
    feedback_cls = getattr(action_type, "Feedback", None)
    send_goal = getattr(action_type, "SendGoal", None)
    get_result = getattr(action_type, "GetResult", None)
    feedback_msg_cls = getattr(action_type, "FeedbackMessage", None)
    cancel_srv = getattr(action_type, "CancelGoal", None)

    if not all([goal_cls, result_cls, feedback_cls, send_goal, get_result, feedback_msg_cls]):
        raise RuntimeError("Action type missing required nested classes (Goal/Result/Feedback/SendGoal/GetResult/FeedbackMessage)")

    send_goal_req = getattr(send_goal, "Request", None)
    send_goal_res = getattr(send_goal, "Response", None)
    get_result_req = getattr(get_result, "Request", None)
    get_result_res = getattr(get_result, "Response", None)
    if not all([send_goal_req, send_goal_res, get_result_req, get_result_res]):
        raise RuntimeError("Action SendGoal/GetResult missing Request/Response")

    if cancel_srv is None:
        raise RuntimeError("Action type missing CancelGoal definition")
    cancel_req = getattr(cancel_srv, "Request", None)
    cancel_res = getattr(cancel_srv, "Response", None)
    if not all([cancel_req, cancel_res]):
        raise RuntimeError("CancelGoal missing Request/Response")

    return {
        "goal": goal_cls,
        "result": result_cls,
        "feedback": feedback_cls,
        "send_goal_req": send_goal_req,
        "send_goal_res": send_goal_res,
        "get_result_req": get_result_req,
        "get_result_res": get_result_res,
        "feedback_msg": feedback_msg_cls,
        "cancel_req": cancel_req,
        "cancel_res": cancel_res,
    }


def _find_first_pubsub(mod, prefer: str | None = None):
    # <Name>PubSubType を優先
    if prefer:
        cand = prefer + "PubSubType"
        if hasattr(mod, cand):
            return getattr(mod, cand)
    # それ以外の *PubSubType でも可
    for n in dir(mod):
        if n.endswith("PubSubType"):
            return getattr(mod, n)
    return None


def _pair_from_module(mod):
    pubsub = _find_first_pubsub(mod)
    if pubsub is None:
        return None, None
    # たとえば StringPubSubType -> String を優先
    base = pubsub.__name__.removesuffix("PubSubType")
    msg_cls = getattr(mod, base, None)
    if isinstance(msg_cls, type):
        return pubsub, msg_cls
    # フォールバック：最初に見つかったクラス
    for n in dir(mod):
        obj = getattr(mod, n)
        if isinstance(obj, type) and not n.endswith("PubSubType"):
            return pubsub, obj
    return pubsub, None


def get_or_create_topic(participant, name: str, type_name: str):
    """
    Reuse an existing Topic on the participant when the name is already registered.
    Returns (topic, owned) where owned=False means an existing Topic was reused.
    """
    import fastdds  # local import to avoid mandatory dependency at import-time

    # Try to reuse an existing Topic instance first
    try:
        duration = fastdds.Duration_t()
        duration.seconds = 0
        duration.nanosec = 0
        existing_topic = participant.find_topic(name, duration)
    except Exception:
        existing_topic = None
    if existing_topic is not None:
        # Ensure type matches
        try:
            existing_type = existing_topic.get_type_name()
            if existing_type and existing_type != type_name:
                raise RuntimeError(
                    f"Topic '{name}' already exists with type '{existing_type}' (requested '{type_name}')"
                )
            return existing_topic, False
        except Exception:
            pass

    tq = fastdds.TopicQos()
    participant.get_default_topic_qos(tq)
    topic_obj = participant.create_topic(name, type_name, tq)
    if topic_obj is None:
        raise RuntimeError(f"Failed to create topic '{name}'")
    return topic_obj, True
