import { MessageEvent, PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

interface LogEntry {
  id: string;
  time: string;
  type: "info" | "success" | "error" | "warn";
  text: string;
}

interface FsdStateMsg {
  config_state?: number;
  flight_phase?: number;
  active_strategy?: string;
  armed?: boolean;
  locked?: boolean;
  ready_for_mode?: boolean;
  mission_id?: string;
  sortie_id?: string;
}

interface ReadinessMsg {
  ready?: boolean;
  readiness_revision?: number;
  failures?: string[];
}

interface PayloadStatusMsg {
  cargo_loaded?: boolean;
  secured?: boolean;
  commanded_state?: number;
  feedback_state?: number;
}

interface LiveTargetLockMsg {
  lock_state?: number;
  quality?: number;
  identity?: {
    marker_id?: number;
    dictionary?: string;
    target_namespace?: string;
  };
}

function FsdMissionControlPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [markerId, setMarkerId] = useState<number>(1);
  const [dictionary, setDictionary] = useState<string>("DICT_4X4_50");
  const [targetNamespace, setTargetNamespace] = useState<string>("aavc2026");

  const [isCalling, setIsCalling] = useState<boolean>(false);
  const [logs, setLogs] = useState<LogEntry[]>([]);

  // Telemetry / State
  const [fsdState, setFsdState] = useState<FsdStateMsg | undefined>();
  const [readiness, setReadiness] = useState<ReadinessMsg | undefined>();
  const [payloadStatus, setPayloadStatus] = useState<PayloadStatusMsg | undefined>();
  const [liveLock, setLiveLock] = useState<LiveTargetLockMsg | undefined>();

  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const addLog = (type: "info" | "success" | "error" | "warn", text: string) => {
    const time = new Date().toLocaleTimeString();
    setLogs((prev) => [{ id: Math.random().toString(), time, type, text }, ...prev.slice(0, 20)]);
  };

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);

      if (renderState.currentFrame) {
        for (const msg of renderState.currentFrame) {
          const m = msg as MessageEvent<unknown>;
          if (m.topic === "/full_self_driving/state") {
            setFsdState(m.message as FsdStateMsg);
          } else if (m.topic === "/full_self_driving/readiness") {
            setReadiness(m.message as ReadinessMsg);
          } else if (m.topic === "/full_self_driving/payload/status") {
            setPayloadStatus(m.message as PayloadStatusMsg);
          } else if (m.topic === "/full_self_driving/perception/live_target_lock") {
            setLiveLock(m.message as LiveTargetLockMsg);
          }
        }
      }
    };

    context.watch("currentFrame");
    context.watch("topics");

    context.subscribe([
      { topic: "/full_self_driving/state" },
      { topic: "/full_self_driving/readiness" },
      { topic: "/full_self_driving/payload/status" },
      { topic: "/full_self_driving/perception/live_target_lock" },
    ]);
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Safe JSON Stringifier for ROS 2 BigInt fields (uint64)
  const safeStringify = (obj: unknown): string => {
    try {
      return JSON.stringify(obj, (_key, value) =>
        typeof value === "bigint" ? value.toString() : value
      );
    } catch {
      return String(obj);
    }
  };

  // Service Caller
  const callRosService = async (serviceName: string, payload: unknown, actionDesc: string) => {
    setIsCalling(true);
    addLog("info", `Calling ${serviceName}...`);

    try {
      const callFn = (context as unknown as { callService?: (name: string, req: unknown) => Promise<unknown> }).callService;
      if (typeof callFn !== "function") {
        throw new Error("Foxglove data source does not support service calls (make sure foxglove_bridge is connected)");
      }

      const res = (await callFn(serviceName, payload)) as Record<string, unknown>;
      addLog("success", `${actionDesc} -> Success: ${safeStringify(res)}`);
    } catch (err: unknown) {
      const msg = err instanceof Error ? err.message : String(err);
      addLog("error", `${actionDesc} -> Error: ${msg}`);
    } finally {
      setIsCalling(false);
    }
  };

  const handleSelectTarget = () => {
    callRosService(
      "/full_self_driving/select_target",
      {
        target: {
          marker_id: markerId,
          dictionary,
          target_namespace: targetNamespace,
        },
        expected_selection_revision: 0,
      },
      `Select Target ID ${markerId}`
    );
  };

  const handlePreparePayload = (operation: number, opName: string) => {
    callRosService(
      "/full_self_driving/prepare_payload",
      {
        request_id: `prep_${Date.now()}`,
        operation,
        expected_selection_revision: 0,
      },
      `Payload: ${opName}`
    );
  };

  const handleEmergencyStop = () => {
    callRosService(
      "/full_self_driving/emergency_stop",
      {
        reason: "Operator manual emergency stop from Foxglove Panel",
      },
      "🚨 EMERGENCY STOP"
    );
  };

  // Lock State Label
  const getLockStateLabel = (state?: number) => {
    switch (state) {
      case 0: return { label: "NO TARGET", color: "#888" };
      case 1: return { label: "ACQUIRING", color: "#f39c12" };
      case 2: return { label: "CANDIDATE", color: "#3498db" };
      case 3: return { label: "QUALIFIED (LOCKED)", color: "#2ecc71" };
      case 4: return { label: "TARGET LOST", color: "#e74c3c" };
      default: return { label: "DISARMED / UNKNOWN", color: "#888" };
    }
  };

  const lockInfo = getLockStateLabel(liveLock?.lock_state);

  return (
    <div style={{ padding: "12px", fontFamily: "system-ui, -apple-system, sans-serif", color: "#fff", background: "#1e1e24", height: "100%", overflowY: "auto", boxSizing: "border-box" }}>
      {/* Header */}
      <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", borderBottom: "1px solid #333", paddingBottom: "8px", marginBottom: "12px" }}>
        <h3 style={{ margin: 0, fontSize: "16px", color: "#58a6ff" }}>🚁 FSD Mission Control</h3>
        <span style={{ fontSize: "11px", padding: "2px 8px", borderRadius: "4px", background: readiness?.ready ? "#238636" : "#da3633", color: "#fff", fontWeight: "bold" }}>
          {readiness?.ready ? "✓ READY FOR MODE" : "✗ NOT READY"}
        </span>
      </div>

      {/* Flight State Overview Bar */}
      <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr 1fr", gap: "8px", background: "#26262e", padding: "8px", borderRadius: "6px", marginBottom: "12px", fontSize: "12px" }}>
        <div>
          <span style={{ color: "#8b949e" }}>Strategy: </span>
          <b style={{ color: "#79c0ff" }}>{fsdState?.active_strategy ?? "IDLE / STANDBY"}</b>
        </div>
        <div>
          <span style={{ color: "#8b949e" }}>Armed: </span>
          <b style={{ color: fsdState?.armed ? "#7ee787" : "#8b949e" }}>{fsdState?.armed ? "YES" : "NO"}</b>
        </div>
        <div>
          <span style={{ color: "#8b949e" }}>Visual Lock: </span>
          <b style={{ color: lockInfo.color }}>{lockInfo.label}</b>
        </div>
      </div>

      {/* 1. Target Selection Section */}
      <div style={{ background: "#26262e", padding: "10px", borderRadius: "6px", marginBottom: "12px" }}>
        <h4 style={{ margin: "0 0 8px 0", fontSize: "13px", color: "#f0883e" }}>1. Target Marker Selection</h4>
        <div style={{ display: "flex", gap: "8px", alignItems: "center", marginBottom: "8px" }}>
          <label style={{ fontSize: "12px", width: "70px" }}>Marker ID:</label>
          <select
            value={markerId}
            onChange={(e) => setMarkerId(Number(e.target.value))}
            style={{ flex: 1, padding: "6px", background: "#161b22", color: "#fff", border: "1px solid #30363d", borderRadius: "4px" }}
          >
            {[1, 2, 3, 4, 5, 6, 7, 8].map((id) => (
              <option key={id} value={id}>
                ArUco ID {id} {id === 5 ? "(Home Origin)" : ""}
              </option>
            ))}
          </select>
        </div>

        <div style={{ display: "flex", gap: "8px", alignItems: "center", marginBottom: "8px" }}>
          <label style={{ fontSize: "12px", width: "70px" }}>Dict:</label>
          <input
            type="text"
            value={dictionary}
            onChange={(e) => setDictionary(e.target.value)}
            style={{ flex: 1, padding: "6px", background: "#161b22", color: "#fff", border: "1px solid #30363d", borderRadius: "4px" }}
          />
        </div>

        <div style={{ display: "flex", gap: "8px", alignItems: "center", marginBottom: "8px" }}>
          <label style={{ fontSize: "12px", width: "70px" }}>Namespace:</label>
          <input
            type="text"
            value={targetNamespace}
            onChange={(e) => setTargetNamespace(e.target.value)}
            style={{ flex: 1, padding: "6px", background: "#161b22", color: "#fff", border: "1px solid #30363d", borderRadius: "4px" }}
          />
        </div>

        <button
          onClick={handleSelectTarget}
          disabled={isCalling}
          style={{ width: "100%", padding: "8px", background: "#1f6feb", color: "#fff", border: "none", borderRadius: "4px", fontWeight: "bold", cursor: isCalling ? "not-allowed" : "pointer" }}
        >
          🎯 Select & Commit Target (ID: {markerId})
        </button>
      </div>

      {/* 2. Payload & Cargo Controls */}
      <div style={{ background: "#26262e", padding: "10px", borderRadius: "6px", marginBottom: "12px" }}>
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: "8px" }}>
          <h4 style={{ margin: 0, fontSize: "13px", color: "#3fb950" }}>2. Payload / Cargo Preparation</h4>
          <span style={{ fontSize: "11px", color: payloadStatus?.secured ? "#7ee787" : "#e3b341" }}>
            {payloadStatus?.secured ? "🔒 SECURED" : "🔓 UNLATCHED"}
          </span>
        </div>
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "8px", marginBottom: "8px" }}>
          <button
            onClick={() => handlePreparePayload(0, "Open for Loading")}
            disabled={isCalling}
            style={{ padding: "6px", background: "#21262d", color: "#c9d1d9", border: "1px solid #30363d", borderRadius: "4px", cursor: "pointer", fontSize: "12px" }}
          >
            Open for Loading (0)
          </button>
          <button
            onClick={() => handlePreparePayload(1, "Verify Secured")}
            disabled={isCalling}
            style={{ padding: "6px", background: "#21262d", color: "#c9d1d9", border: "1px solid #30363d", borderRadius: "4px", cursor: "pointer", fontSize: "12px" }}
          >
            Verify Secured (1)
          </button>
        </div>
        <button
          onClick={() => handlePreparePayload(2, "Prepare for Sortie")}
          disabled={isCalling}
          style={{ width: "100%", padding: "8px", background: "#238636", color: "#fff", border: "none", borderRadius: "4px", fontWeight: "bold", cursor: "pointer" }}
        >
          📦 Prepare Cargo for Sortie (2)
        </button>
      </div>

      {/* 3. Safety Emergency Stop */}
      <div style={{ background: "#3d1418", padding: "10px", border: "1px solid #f85149", borderRadius: "6px", marginBottom: "12px" }}>
        <button
          onClick={handleEmergencyStop}
          disabled={isCalling}
          style={{ width: "100%", padding: "12px", background: "#da3633", color: "#fff", border: "none", borderRadius: "6px", fontWeight: "bold", fontSize: "14px", cursor: "pointer", letterSpacing: "1px" }}
        >
          🚨 EMERGENCY STOP (E-STOP)
        </button>
      </div>

      {/* Service Call Feedback Log */}
      <div style={{ background: "#161b22", border: "1px solid #30363d", borderRadius: "6px", padding: "8px", maxHeight: "140px", overflowY: "auto" }}>
        <div style={{ fontSize: "11px", color: "#8b949e", marginBottom: "4px", fontWeight: "bold" }}>SERVICE LOGS:</div>
        {logs.length === 0 ? (
          <div style={{ fontSize: "11px", color: "#484f58", fontStyle: "italic" }}>No service calls sent yet.</div>
        ) : (
          logs.map((log) => (
            <div key={log.id} style={{ fontSize: "11px", marginBottom: "3px", color: log.type === "success" ? "#7ee787" : log.type === "error" ? "#f85149" : log.type === "warn" ? "#e3b341" : "#8b949e" }}>
              [{log.time}] {log.text}
            </div>
          ))
        )}
      </div>
    </div>
  );
}

export function initFsdMissionControlPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<FsdMissionControlPanel context={context} />);

  return () => {
    root.unmount();
  };
}
