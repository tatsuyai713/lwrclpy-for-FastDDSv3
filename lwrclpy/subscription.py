import fastdds
from .qos import QoSProfile

def _retcode_is_ok(rc) -> bool:
    """fastdds の戻り値を v2/v3 差を吸収して OK か判定。"""
    ok_const = getattr(fastdds, "RETCODE_OK", 0)  # v3 はモジュール直下, v2 は 0 相当
    try:
        # まず直接比較（列挙/オブジェクトでも True になる実装が多い）
        if rc == ok_const:
            return True
    except Exception:
        pass
    # 数値比較フォールバック
    try:
        return int(rc) == int(ok_const)
    except Exception:
        # 一部で bool True を返す場合（旧実装対策）
        return bool(rc) is True


class _ReaderListener(fastdds.DataReaderListener):
    def __init__(self, on_msg, msg_ctor):
        super().__init__()
        self._on_msg = on_msg
        self._msg_ctor = msg_ctor

    def on_data_available(self, reader):
        info = fastdds.SampleInfo()
        data = self._msg_ctor()

        # 署名や戻り値の差を吸収して take_next_sample を呼ぶ
        try:
            rc = reader.take_next_sample(data, info)
        except TypeError:
            # まれに引数順が違う実装がある
            rc = reader.take_next_sample(info, data)
        except Exception:
            # ここで例外を潰してしまうとデバッグしづらいが、
            # Director の二重例外で abort しないように防御
            return

        if _retcode_is_ok(rc) and getattr(info, "valid_data", True):
            try:
                self._on_msg(data)
            except Exception:
                # ユーザコールバック内の例外は握りつぶす（ログが欲しければここで print）
                return


class Subscription:
    def __init__(self, participant, topic, qos: QoSProfile, callback, msg_ctor):
        sqos = fastdds.SubscriberQos()
        participant.get_default_subscriber_qos(sqos)
        self._participant = participant
        self._subscriber = participant.create_subscriber(sqos)
        rq = fastdds.DataReaderQos()
        self._subscriber.get_default_datareader_qos(rq)
        qos.apply_to_reader(rq)
        self._reader = self._subscriber.create_datareader(topic, rq)
        self._listener = _ReaderListener(callback, msg_ctor)
        # DataReader にリスナを関連付け
        try:
            self._reader.set_listener(self._listener)
        except AttributeError:
            # 旧バインディングでは set_listener が無い場合がある（その時は create_datareader で渡す必要あり）
            # ここでは何もしない（必要なら node 側の create で listener を渡す実装に切替可能）
            pass

    def destroy(self):
        if getattr(self, "_reader", None):
            try:
                self._subscriber.delete_datareader(self._reader)
            except Exception:
                pass
            self._reader = None
        if getattr(self, "_subscriber", None):
            try:
                self._participant.delete_subscriber(self._subscriber)
            except Exception:
                pass
            self._subscriber = None