"""ユーティリティ（生成型の解決をモジュール/クラス両対応に）。"""
import importlib
import types


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