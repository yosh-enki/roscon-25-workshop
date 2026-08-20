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

interface TelemetryMsg {
  armed?: boolean;
  airborne?: boolean;
  landed?: boolean;
  altitude_m?: number;
  ground_speed_m_s?: number;
  battery_percentage?: number;
  heading_deg?: number;
}

const customStyles = `
  @keyframes pulseGlow {
    0% { box-shadow: 0 0 4px rgba(46, 204, 113, 0.4); }
    50% { box-shadow: 0 0 14px rgba(46, 204, 113, 0.8); }
    100% { box-shadow: 0 0 4px rgba(46, 204, 113, 0.4); }
  }
  @keyframes warningGlow {
    0% { box-shadow: 0 0 4px rgba(231, 76, 60, 0.4); }
    50% { box-shadow: 0 0 16px rgba(231, 76, 60, 0.8); }
    100% { box-shadow: 0 0 4px rgba(231, 76, 60, 0.4); }
  }
  .gcs-btn {
    transition: all 0.15s cubic-bezier(0.4, 0, 0.2, 1);
    font-family: inherit;
  }
  .gcs-btn:hover:not(:disabled) {
    transform: translateY(-1px);
    filter: brightness(1.15);
  }
  .gcs-btn:active:not(:disabled) {
    transform: translateY(1px);
    filter: brightness(0.9);
  }
  .gcs-chip {
    transition: all 0.15s ease;
  }
  .gcs-chip:hover {
    transform: scale(1.05);
  }
  .custom-scrollbar::-webkit-scrollbar {
    width: 6px;
    height: 6px;
  }
  .custom-scrollbar::-webkit-scrollbar-track {
    background: rgba(0, 0, 0, 0.2);
    border-radius: 4px;
  }
  .custom-scrollbar::-webkit-scrollbar-thumb {
    background: rgba(255, 255, 255, 0.15);
    border-radius: 4px;
  }
  .custom-scrollbar::-webkit-scrollbar-thumb:hover {
    background: rgba(255, 255, 255, 0.3);
  }
`;

function FsdMissionControlPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [markerId, setMarkerId] = useState<number>(1);
  const dictionary = "DICT_4X4_50";
  const targetNamespace = "aavc2026";

  const [isCalling, setIsCalling] = useState<boolean>(false);
  const [logs, setLogs] = useState<LogEntry[]>([]);

  // Telemetry / State
  const [fsdState, setFsdState] = useState<FsdStateMsg | undefined>();
  const [readiness, setReadiness] = useState<ReadinessMsg | undefined>();
  const [payloadStatus, setPayloadStatus] = useState<PayloadStatusMsg | undefined>();
  const [liveLock, setLiveLock] = useState<LiveTargetLockMsg | undefined>();
  const [telemetry, setTelemetry] = useState<TelemetryMsg | undefined>();

  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const addLog = (type: "info" | "success" | "error" | "warn", text: string) => {
    const time = new Date().toLocaleTimeString();
    setLogs((prev) => [{ id: Math.random().toString(), time, type, text }, ...prev.slice(0, 30)]);
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
          } else if (m.topic === "/full_self_driving/telemetry") {
            setTelemetry(m.message as TelemetryMsg);
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
      { topic: "/full_self_driving/telemetry" },
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
    addLog("info", `⚡ Requesting ${serviceName}...`);

    try {
      const callFn = (context as unknown as { callService?: (name: string, req: unknown) => Promise<unknown> }).callService;
      if (typeof callFn !== "function") {
        throw new Error("Foxglove data source does not support service calls (check foxglove_bridge connection)");
      }

      const res = (await callFn(serviceName, payload)) as Record<string, unknown>;
      addLog("success", `✓ ${actionDesc} -> Accepted: ${safeStringify(res)}`);
    } catch (err: unknown) {
      const msg = err instanceof Error ? err.message : String(err);
      addLog("error", `✗ ${actionDesc} -> Failed: ${msg}`);
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
      `Target Assigned (ID: ${markerId})`
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
      `Payload ${opName}`
    );
  };

  const handleEmergencyStop = () => {
    callRosService(
      "/full_self_driving/emergency_stop",
      {
        reason: "Operator manual emergency stop from Foxglove Panel",
      },
      "🚨 EMERGENCY STOP TRIGGERED"
    );
  };

  // Lock State Label
  const getLockStateInfo = (state?: number) => {
    switch (state) {
      case 0: return { label: "NO TARGET", bg: "rgba(136,136,136,0.2)", border: "#666", text: "#aaa" };
      case 1: return { label: "ACQUIRING", bg: "rgba(243,156,18,0.2)", border: "#f39c12", text: "#f1c40f" };
      case 2: return { label: "CANDIDATE", bg: "rgba(52,152,219,0.2)", border: "#3498db", text: "#5dade2" };
      case 3: return { label: "QUALIFIED (LOCKED)", bg: "rgba(46,204,113,0.2)", border: "#2ecc71", text: "#2ecc71" };
      case 4: return { label: "TARGET LOST", bg: "rgba(231,76,60,0.2)", border: "#e74c3c", text: "#e74c3c" };
      default: return { label: "STANDBY", bg: "rgba(255,255,255,0.05)", border: "#444", text: "#888" };
    }
  };

  const lockInfo = getLockStateInfo(liveLock?.lock_state);
  const isReady = readiness?.ready ?? false;
  const isArmed = fsdState?.armed ?? false;

  return (
    <div
      className="custom-scrollbar"
      style={{
        padding: "16px",
        fontFamily: "'JetBrains Mono', 'Segoe UI', system-ui, -apple-system, sans-serif",
        color: "#e6edf3",
        background: "linear-gradient(180deg, #0d1117 0%, #161b22 100%)",
        height: "100%",
        overflowY: "auto",
        boxSizing: "border-box",
      }}
    >
      <style>{customStyles}</style>

      {/* 🚀 Header: Modern Aerospace HUD Bar */}
      <div
        style={{
          display: "flex",
          justifyContent: "space-between",
          alignItems: "center",
          background: "rgba(22, 27, 34, 0.8)",
          backdropFilter: "blur(10px)",
          border: "1px solid rgba(88, 166, 255, 0.2)",
          borderRadius: "10px",
          padding: "10px 14px",
          marginBottom: "14px",
          boxShadow: "0 4px 20px rgba(0, 0, 0, 0.3)",
        }}
      >
        <div style={{ display: "flex", alignItems: "center", gap: "10px" }}>
          <div
            style={{
              width: "10px",
              height: "10px",
              borderRadius: "50%",
              background: isReady ? "#2ea043" : "#f85149",
              animation: isReady ? "pulseGlow 2s infinite" : "none",
            }}
          />
          <div>
            <div style={{ fontSize: "14px", fontWeight: "bold", letterSpacing: "0.5px", color: "#58a6ff" }}>
              FSD MISSION COMMAND
            </div>
            <div style={{ fontSize: "10px", color: "#8b949e" }}>ROS 2 AUTONOMOUS FLIGHT CONTROLLER</div>
          </div>
        </div>

        <div
          style={{
            fontSize: "11px",
            padding: "4px 12px",
            borderRadius: "20px",
            background: isReady ? "rgba(46, 160, 67, 0.15)" : "rgba(248, 81, 73, 0.15)",
            border: `1px solid ${isReady ? "#2ea043" : "#f85149"}`,
            color: isReady ? "#3fb950" : "#f85149",
            fontWeight: "bold",
            letterSpacing: "0.5px",
          }}
        >
          {isReady ? "✓ READY FOR MODE" : "✗ PRE-FLIGHT NOT READY"}
        </div>
      </div>

      {/* 📊 HUD Quick Metrics Cards */}
      <div
        style={{
          display: "grid",
          gridTemplateColumns: "repeat(3, 1fr)",
          gap: "8px",
          marginBottom: "14px",
        }}
      >
        {/* Strategy Card */}
        <div
          style={{
            background: "rgba(33, 38, 45, 0.7)",
            border: "1px solid rgba(48, 54, 61, 0.8)",
            borderRadius: "8px",
            padding: "8px 10px",
          }}
        >
          <div style={{ fontSize: "10px", color: "#8b949e", textTransform: "uppercase", marginBottom: "2px" }}>Strategy</div>
          <div style={{ fontSize: "12px", fontWeight: "bold", color: "#79c0ff", textOverflow: "ellipsis", overflow: "hidden", whiteSpace: "nowrap" }}>
            {fsdState?.active_strategy || "STANDBY"}
          </div>
        </div>

        {/* Armed Card */}
        <div
          style={{
            background: "rgba(33, 38, 45, 0.7)",
            border: `1px solid ${isArmed ? "rgba(46,160,67,0.4)" : "rgba(48, 54, 61, 0.8)"}`,
            borderRadius: "8px",
            padding: "8px 10px",
          }}
        >
          <div style={{ fontSize: "10px", color: "#8b949e", textTransform: "uppercase", marginBottom: "2px" }}>Armed State</div>
          <div style={{ fontSize: "12px", fontWeight: "bold", color: isArmed ? "#3fb950" : "#8b949e" }}>
            {isArmed ? "ARMED (LOCKED)" : "DISARMED"}
          </div>
        </div>

        {/* Visual Lock Card */}
        <div
          style={{
            background: lockInfo.bg,
            border: `1px solid ${lockInfo.border}`,
            borderRadius: "8px",
            padding: "8px 10px",
          }}
        >
          <div style={{ fontSize: "10px", color: "#8b949e", textTransform: "uppercase", marginBottom: "2px" }}>ArUco Lock</div>
          <div style={{ fontSize: "11px", fontWeight: "bold", color: lockInfo.text, textOverflow: "ellipsis", overflow: "hidden", whiteSpace: "nowrap" }}>
            {lockInfo.label}
          </div>
        </div>
      </div>

      {/* 🧭 Telemetry Real-time Strip */}
      {telemetry && (
        <div
          style={{
            display: "grid",
            gridTemplateColumns: "1fr 1fr 1fr",
            gap: "8px",
            background: "rgba(22, 27, 34, 0.5)",
            border: "1px dashed rgba(48, 54, 61, 0.8)",
            borderRadius: "8px",
            padding: "8px 10px",
            marginBottom: "14px",
            fontSize: "11px",
          }}
        >
          <div>
            <span style={{ color: "#8b949e" }}>Alt: </span>
            <b style={{ color: "#58a6ff" }}>{(telemetry.altitude_m ?? 0).toFixed(2)} m</b>
          </div>
          <div>
            <span style={{ color: "#8b949e" }}>Speed: </span>
            <b style={{ color: "#e3b341" }}>{(telemetry.ground_speed_m_s ?? 0).toFixed(1)} m/s</b>
          </div>
          <div>
            <span style={{ color: "#8b949e" }}>Battery: </span>
            <b style={{ color: (telemetry.battery_percentage ?? 100) > 30 ? "#3fb950" : "#f85149" }}>
              {(telemetry.battery_percentage ?? 100).toFixed(0)}%
            </b>
          </div>
        </div>
      )}

      {/* 🎯 Section 1: Target Selection Card */}
      <div
        style={{
          background: "rgba(22, 27, 34, 0.8)",
          border: "1px solid #30363d",
          borderRadius: "10px",
          padding: "14px",
          marginBottom: "14px",
          boxShadow: "0 4px 12px rgba(0, 0, 0, 0.2)",
        }}
      >
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: "10px" }}>
          <div style={{ fontSize: "12px", fontWeight: "bold", color: "#f0883e", letterSpacing: "0.5px" }}>
            1. TARGET ARUCO SELECTION
          </div>
          <span style={{ fontSize: "10px", background: "rgba(240, 136, 62, 0.15)", color: "#f0883e", padding: "2px 6px", borderRadius: "4px" }}>
            DICT_4X4_50
          </span>
        </div>

        {/* Quick Marker Chip Buttons */}
        <div style={{ fontSize: "11px", color: "#8b949e", marginBottom: "6px" }}>Select Pad ID:</div>
        <div style={{ display: "flex", flexWrap: "wrap", gap: "6px", marginBottom: "12px" }}>
          {[
            { id: 1, label: "Pad 1" },
            { id: 2, label: "Pad 2" },
            { id: 3, label: "Pad 3" },
            { id: 4, label: "Pad 4" },
            { id: 5, label: "Pad 5" },
            { id: 6, label: "Pad 6" },
          ].map((item) => {
            const isSelected = markerId === item.id;
            return (
              <button
                key={item.id}
                type="button"
                className="gcs-chip"
                onClick={() => setMarkerId(item.id)}
                style={{
                  padding: "6px 10px",
                  borderRadius: "6px",
                  fontSize: "11px",
                  fontWeight: isSelected ? "bold" : "normal",
                  background: isSelected ? "linear-gradient(135deg, #1f6feb 0%, #388bfd 100%)" : "rgba(33, 38, 45, 0.8)",
                  border: isSelected ? "1px solid #58a6ff" : "1px solid #30363d",
                  color: isSelected ? "#fff" : "#c9d1d9",
                  cursor: "pointer",
                  boxShadow: isSelected ? "0 0 10px rgba(31, 111, 235, 0.5)" : "none",
                }}
              >
                {item.label}
              </button>
            );
          })}
        </div>

        <button
          type="button"
          className="gcs-btn"
          onClick={handleSelectTarget}
          disabled={isCalling}
          style={{
            width: "100%",
            padding: "10px",
            background: "linear-gradient(135deg, #238636 0%, #2ea043 100%)",
            color: "#fff",
            border: "1px solid rgba(255, 255, 255, 0.2)",
            borderRadius: "6px",
            fontWeight: "bold",
            fontSize: "12px",
            letterSpacing: "0.5px",
            cursor: isCalling ? "not-allowed" : "pointer",
            boxShadow: "0 2px 8px rgba(46, 160, 67, 0.4)",
          }}
        >
          ASSIGN PAD {markerId}
        </button>
      </div>

      {/* 📦 Section 2: Cargo & Payload Preparation */}
      <div
        style={{
          background: "rgba(22, 27, 34, 0.8)",
          border: "1px solid #30363d",
          borderRadius: "10px",
          padding: "14px",
          marginBottom: "14px",
          boxShadow: "0 4px 12px rgba(0, 0, 0, 0.2)",
        }}
      >
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: "10px" }}>
          <div style={{ fontSize: "12px", fontWeight: "bold", color: "#3fb950", letterSpacing: "0.5px" }}>
            2. PAYLOAD & CARGO MECHANISM
          </div>
          <span
            style={{
              fontSize: "10px",
              padding: "2px 8px",
              borderRadius: "4px",
              background: payloadStatus?.secured ? "rgba(46,160,67,0.15)" : "rgba(227,179,65,0.15)",
              color: payloadStatus?.secured ? "#3fb950" : "#e3b341",
              border: `1px solid ${payloadStatus?.secured ? "#2ea043" : "#d29922"}`,
              fontWeight: "bold",
            }}
          >
            {payloadStatus?.secured ? "SECURED & LOCKED" : "UNLATCHED"}
          </span>
        </div>

        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "8px", marginBottom: "8px" }}>
          <button
            type="button"
            className="gcs-btn"
            onClick={() => handlePreparePayload(0, "Open for Loading")}
            disabled={isCalling}
            style={{
              padding: "8px",
              background: "rgba(33, 38, 45, 0.8)",
              color: "#c9d1d9",
              border: "1px solid #30363d",
              borderRadius: "6px",
              cursor: "pointer",
              fontSize: "11px",
              fontWeight: "bold",
            }}
          >
            OPEN (0)
          </button>
          <button
            type="button"
            className="gcs-btn"
            onClick={() => handlePreparePayload(1, "Verify Secured")}
            disabled={isCalling}
            style={{
              padding: "8px",
              background: "rgba(33, 38, 45, 0.8)",
              color: "#c9d1d9",
              border: "1px solid #30363d",
              borderRadius: "6px",
              cursor: "pointer",
              fontSize: "11px",
              fontWeight: "bold",
            }}
          >
            VERIFY SENSOR (1)
          </button>
        </div>

        <button
          type="button"
          className="gcs-btn"
          onClick={() => handlePreparePayload(2, "Close & Lock for Sortie")}
          disabled={isCalling}
          style={{
            width: "100%",
            padding: "10px",
            background: "linear-gradient(135deg, #1f6feb 0%, #388bfd 100%)",
            color: "#fff",
            border: "1px solid rgba(255, 255, 255, 0.2)",
            borderRadius: "6px",
            fontWeight: "bold",
            fontSize: "12px",
            letterSpacing: "0.5px",
            cursor: isCalling ? "not-allowed" : "pointer",
            boxShadow: "0 2px 8px rgba(31, 111, 235, 0.4)",
          }}
        >
          CLOSE & LOCK (2)
        </button>
      </div>

      {/* 🚨 Section 3: Tactical Emergency Stop */}
      <div
        style={{
          background: "linear-gradient(180deg, rgba(218, 54, 51, 0.15) 0%, rgba(30, 10, 10, 0.8) 100%)",
          border: "1px solid #f85149",
          borderRadius: "10px",
          padding: "12px",
          marginBottom: "14px",
          boxShadow: "0 4px 16px rgba(248, 81, 73, 0.2)",
        }}
      >
        <button
          type="button"
          className="gcs-btn"
          onClick={handleEmergencyStop}
          disabled={isCalling}
          style={{
            width: "100%",
            padding: "12px",
            background: "linear-gradient(135deg, #da3633 0%, #f85149 100%)",
            color: "#fff",
            border: "1px solid rgba(255, 255, 255, 0.3)",
            borderRadius: "8px",
            fontWeight: "900",
            fontSize: "13px",
            letterSpacing: "1.5px",
            cursor: "pointer",
            boxShadow: "0 0 16px rgba(248, 81, 73, 0.5)",
            animation: "warningGlow 3s infinite",
          }}
        >
          EMERGENCY STOP (E-STOP)
        </button>
      </div>

      {/* 💻 Section 4: Live Service Console Logs */}
      <div
        style={{
          background: "#0d1117",
          border: "1px solid #30363d",
          borderRadius: "8px",
          padding: "10px",
        }}
      >
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: "6px" }}>
          <div style={{ fontSize: "10px", fontWeight: "bold", color: "#8b949e", letterSpacing: "0.5px" }}>
            SERVICE AUDIT TRAIL
          </div>
          {logs.length > 0 && (
            <button
              type="button"
              onClick={() => setLogs([])}
              style={{ fontSize: "9px", background: "none", border: "none", color: "#58a6ff", cursor: "pointer" }}
            >
              Clear
            </button>
          )}
        </div>

        <div className="custom-scrollbar" style={{ maxHeight: "120px", overflowY: "auto", fontSize: "10px", lineHeight: "1.4" }}>
          {logs.length === 0 ? (
            <div style={{ color: "#484f58", fontStyle: "italic" }}>System standby. No service requests dispatched yet.</div>
          ) : (
            logs.map((log) => (
              <div
                key={log.id}
                style={{
                  marginBottom: "4px",
                  padding: "2px 4px",
                  borderRadius: "3px",
                  background: log.type === "success" ? "rgba(46,160,67,0.1)" : log.type === "error" ? "rgba(248,81,73,0.1)" : "rgba(255,255,255,0.02)",
                  color: log.type === "success" ? "#7ee787" : log.type === "error" ? "#ff7b72" : log.type === "warn" ? "#d29922" : "#8b949e",
                  borderLeft: `2px solid ${log.type === "success" ? "#2ea043" : log.type === "error" ? "#f85149" : "#58a6ff"}`,
                }}
              >
                <span style={{ color: "#484f58", marginRight: "6px" }}>[{log.time}]</span>
                {log.text}
              </div>
            ))
          )}
        </div>
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
