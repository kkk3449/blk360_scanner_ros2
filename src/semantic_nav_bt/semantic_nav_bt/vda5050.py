"""VDA 5050 v2.0 message helpers + a thin MQTT wrapper shared by the console-side
master control (vda5050_master), the AGV adapter (vda5050_agv_adapter) and the
Isaac twin (scripts/isaacsim_kg_twin.py imports the same builders by path).

Topic layout (VDA 5050 §6.1):
    {interfaceName}/{majorVersion}/{manufacturer}/{serialNumber}/{topic}
    e.g.  uagv/v2/caselab/ammr20/order

Topics used here: order, instantActions (master -> AGV);
                  state, visualization, connection (AGV -> master).
Only the subset of fields this system needs is populated, but every message
carries the mandatory header (headerId, timestamp, version, manufacturer,
serialNumber) and the mandatory structural fields of its schema, so a
standard VDA 5050 master control or AGV can be swapped in.
"""
import datetime as _dt
import itertools
import json
import threading

VERSION = "2.0.0"
INTERFACE = "uagv"
MAJOR = "v2"
TOPICS = ("order", "instantActions", "state", "visualization", "connection")


def topic(manufacturer, serial, name):
    return f"{INTERFACE}/{MAJOR}/{manufacturer}/{serial}/{name}"


def parse_topic(t):
    """-> (manufacturer, serial, name) or None."""
    p = t.split("/")
    if len(p) == 5 and p[0] == INTERFACE and p[1] == MAJOR and p[4] in TOPICS:
        return p[2], p[3], p[4]
    return None


def now_iso():
    return _dt.datetime.now(_dt.timezone.utc).isoformat(
        timespec="milliseconds").replace("+00:00", "Z")


class Header:
    """Per-topic monotonically increasing headerId."""

    def __init__(self, manufacturer, serial):
        self.m, self.s = manufacturer, serial
        self._c = {}

    def __call__(self, name):
        n = self._c.setdefault(name, itertools.count())
        return {"headerId": next(n), "timestamp": now_iso(), "version": VERSION,
                "manufacturer": self.m, "serialNumber": self.s}


# ---------------------------------------------------------------- builders --
def node(node_id, seq, x, y, theta, map_id, released=True, dev_xy=0.35,
         dev_theta=3.14159, actions=None, description=""):
    return {"nodeId": node_id, "sequenceId": seq, "released": released,
            "nodeDescription": description,
            "nodePosition": {"x": float(x), "y": float(y),
                             "theta": float(theta), "mapId": map_id,
                             "allowedDeviationXY": float(dev_xy),
                             "allowedDeviationTheta": float(dev_theta)},
            "actions": actions or []}


def edge(edge_id, seq, start, end, released=True, max_speed=None,
         actions=None):
    e = {"edgeId": edge_id, "sequenceId": seq, "released": released,
         "startNodeId": start, "endNodeId": end, "actions": actions or []}
    if max_speed is not None:
        e["maxSpeed"] = float(max_speed)
    return e


def make_order(hdr, order_id, order_update_id, nodes, edges):
    m = hdr("order")
    m.update({"orderId": order_id, "orderUpdateId": int(order_update_id),
              "nodes": nodes, "edges": edges})
    return m


def action(action_type, action_id, blocking="HARD", params=None,
           description=""):
    return {"actionType": action_type, "actionId": action_id,
            "blockingType": blocking, "actionDescription": description,
            "actionParameters": [{"key": k, "value": v}
                                 for k, v in (params or {}).items()]}


def make_instant_actions(hdr, actions):
    m = hdr("instantActions")
    m["actions"] = list(actions)
    return m


def make_connection(hdr, state):
    m = hdr("connection")
    m["connectionState"] = state          # ONLINE | OFFLINE | CONNECTIONBROKEN
    return m


def make_visualization(hdr, x, y, theta, map_id, vx=0.0, vy=0.0, omega=0.0):
    m = hdr("visualization")
    m["agvPosition"] = {"x": float(x), "y": float(y), "theta": float(theta),
                        "mapId": map_id, "positionInitialized": True}
    m["velocity"] = {"vx": float(vx), "vy": float(vy), "omega": float(omega)}
    return m


def make_state(hdr, *, order_id="", order_update_id=0, last_node_id="",
               last_node_seq=0, node_states=(), edge_states=(),
               agv_position=None, velocity=None, driving=False, paused=False,
               action_states=(), battery_charge=100.0, charging=False,
               operating_mode="AUTOMATIC", errors=(), information=(),
               new_base_request=False, e_stop="NONE", field_violation=False,
               distance_since_last_node=None):
    m = hdr("state")
    m.update({
        "orderId": order_id, "orderUpdateId": int(order_update_id),
        "lastNodeId": last_node_id, "lastNodeSequenceId": int(last_node_seq),
        "nodeStates": list(node_states), "edgeStates": list(edge_states),
        "driving": bool(driving), "paused": bool(paused),
        "newBaseRequest": bool(new_base_request),
        "actionStates": list(action_states),
        "batteryState": {"batteryCharge": float(battery_charge),
                         "charging": bool(charging)},
        "operatingMode": operating_mode,
        "errors": list(errors), "information": list(information),
        "safetyState": {"eStop": e_stop, "fieldViolation": bool(field_violation)},
    })
    if agv_position is not None:
        m["agvPosition"] = agv_position
    if velocity is not None:
        m["velocity"] = velocity
    if distance_since_last_node is not None:
        m["distanceSinceLastNode"] = float(distance_since_last_node)
    return m


def error(error_type, level="WARNING", description="", refs=None):
    return {"errorType": error_type, "errorLevel": level,
            "errorDescription": description,
            "errorReferences": [{"referenceKey": k, "referenceValue": str(v)}
                                for k, v in (refs or {}).items()]}


def action_state(action_id, action_type, status, description=""):
    # status: WAITING | INITIALIZING | RUNNING | PAUSED | FINISHED | FAILED
    return {"actionId": action_id, "actionType": action_type,
            "actionStatus": status, "actionDescription": description}


# -------------------------------------------------------------------- MQTT --
class Mqtt:
    """paho wrapper: JSON publish/subscribe, background loop, optional LWT.
    Callbacks get (manufacturer, serial, topic_name, payload_dict)."""

    def __init__(self, host="127.0.0.1", port=1883, client_id="",
                 will=None, on_connect=None, log=print):
        import paho.mqtt.client as mq
        self._mq = mq
        self.c = mq.Client(mq.CallbackAPIVersion.VERSION2, client_id=client_id,
                           clean_session=True)
        self.host, self.port, self.log = host, int(port), log
        self._subs = []                 # (topic filter, callback)
        self._lock = threading.Lock()
        self.connected = threading.Event()
        self._user_on_connect = on_connect
        if will is not None:            # (topic, payload_dict)
            self.c.will_set(will[0], json.dumps(will[1]), qos=1, retain=True)
        self.c.on_connect = self._on_connect
        self.c.on_disconnect = self._on_disconnect
        self.c.on_message = self._on_message

    def start(self):
        self.c.connect_async(self.host, self.port, keepalive=10)
        self.c.loop_start()

    def stop(self):
        try:
            self.c.disconnect()
        finally:
            self.c.loop_stop()

    def subscribe(self, topic_filter, cb):
        with self._lock:
            self._subs.append((topic_filter, cb))
        if self.connected.is_set():
            self.c.subscribe(topic_filter, qos=1)

    def publish(self, topic_, payload, retain=False, qos=1):
        return self.c.publish(topic_, json.dumps(payload), qos=qos,
                              retain=retain)

    # -- paho callbacks
    def _on_connect(self, c, userdata, flags, reason, props=None):
        self.log(f"[mqtt] connected {self.host}:{self.port} rc={reason}")
        with self._lock:
            for t, _ in self._subs:
                c.subscribe(t, qos=1)
        self.connected.set()
        if self._user_on_connect:
            self._user_on_connect()

    def _on_disconnect(self, c, userdata, flags, reason, props=None):
        self.connected.clear()
        self.log(f"[mqtt] disconnected rc={reason}")

    def _on_message(self, c, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except ValueError:
            self.log(f"[mqtt] non-JSON on {msg.topic}")
            return
        pt = parse_topic(msg.topic)
        with self._lock:
            subs = list(self._subs)
        for filt, cb in subs:
            if self._mq.topic_matches_sub(filt, msg.topic):
                try:
                    cb(pt, payload)
                except Exception as e:  # noqa: BLE001
                    self.log(f"[mqtt] handler error on {msg.topic}: {e!r}")
