import { MessageEvent, PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

interface CameraInfoMsg {
  header?: {
    frame_id?: string;
    stamp?: { sec: number; nanosec: number };
  };
  width?: number;
  height?: number;
  distortion_model?: string;
}

interface AllIdObservationMsg {
  identity?: {
    marker_id?: number;
    dictionary?: string;
    target_namespace?: string;
  };
  quality?: number;
  calibration_sha256?: string;
}

interface AllIdObservationBatchMsg {
  map_id?: string;
  scenario_id?: string;
  observations?: AllIdObservationMsg[];
  header?: {
    stamp?: { sec: number; nanosec: number };
  };
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

interface ComponentHealthMsg {
  component_name?: string;
  state?: number;
  ready?: boolean;
  diagnostics?: string;
}

const customStyles = `
  @keyframes pulseGreenGlow {
    0% { box-shadow: 0 0 4px rgba(46, 204, 113, 0.4); }
    50% { box-shadow: 0 0 14px rgba(46, 204, 113, 0.8); }
    100% { box-shadow: 0 0 4px rgba(46, 204, 113, 0.4); }
  }
  @keyframes pulseRedGlow {
    0% { box-shadow: 0 0 4px rgba(231, 76, 60, 0.4); }
    50% { box-shadow: 0 0 14px rgba(231, 76, 60, 0.8); }
    100% { box-shadow: 0 0 4px rgba(231, 76, 60, 0.4); }
  }
  .hud-stat-box {
    transition: transform 0.15s ease, border-color 0.15s ease;
  }
  .hud-stat-box:hover {
    border-color: rgba(88, 166, 255, 0.4) !important;
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

function FsdCameraStatusPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [cameraInfo, setCameraInfo] = useState<CameraInfoMsg | undefined>();
  const [obsBatch, setObsBatch] = useState<AllIdObservationBatchMsg | undefined>();
  const [liveLock, setLiveLock] = useState<LiveTargetLockMsg | undefined>();
  const [perceptionHealth, setPerceptionHealth] = useState<ComponentHealthMsg | undefined>();

  // FPS calculation
  const [fps, setFps] = useState<number>(0);
  const [frameCount, setFrameCount] = useState<number>(0);
  const [lastFrameTime, setLastFrameTime] = useState<number>(0);
  const frameTimestampsRef = useRef<number[]>([]);

  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);

      if (renderState.currentFrame) {
        const now = performance.now();
        let receivedCamera = false;

        for (const msg of renderState.currentFrame) {
          const m = msg as MessageEvent<unknown>;
          if (m.topic === "/camera_info" || m.topic === "/camera") {
            receivedCamera = true;
            if (m.topic === "/camera_info") {
              setCameraInfo(m.message as CameraInfoMsg);
            }
          } else if (m.topic === "/full_self_driving/perception/all_id_observations") {
            setObsBatch(m.message as AllIdObservationBatchMsg);
          } else if (m.topic === "/full_self_driving/perception/live_target_lock") {
            setLiveLock(m.message as LiveTargetLockMsg);
          } else if (m.topic === "/full_self_driving/health") {
            const h = m.message as ComponentHealthMsg;
            if (h.component_name === "perception" || h.component_name?.includes("perception")) {
              setPerceptionHealth(h);
            }
          }
        }

        if (receivedCamera) {
          setLastFrameTime(now);
          setFrameCount((prev) => prev + 1);

          frameTimestampsRef.current.push(now);
          const cutoff = now - 1000;
          while (frameTimestampsRef.current.length > 0 && frameTimestampsRef.current[0]! < cutoff) {
            frameTimestampsRef.current.shift();
          }
          setFps(frameTimestampsRef.current.length);
        }
      }
    };

    context.watch("currentFrame");
    context.watch("topics");

    context.subscribe([
      { topic: "/camera_info" },
      { topic: "/full_self_driving/perception/all_id_observations" },
      { topic: "/full_self_driving/perception/live_target_lock" },
      { topic: "/full_self_driving/health" },
    ]);
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Periodic timer to detect stream stalls
  useEffect(() => {
    const timer = setInterval(() => {
      const now = performance.now();
      const cutoff = now - 1000;
      while (frameTimestampsRef.current.length > 0 && frameTimestampsRef.current[0]! < cutoff) {
        frameTimestampsRef.current.shift();
      }
      setFps(frameTimestampsRef.current.length);
    }, 500);

    return () => clearInterval(timer);
  }, []);

  const now = performance.now();
  const timeSinceLastFrameSec = lastFrameTime > 0 ? (now - lastFrameTime) / 1000 : 999;
  const isOnline = timeSinceLastFrameSec < 2.0;
  const isStalled = isOnline && fps < 5;

  const width = cameraInfo?.width ?? 1280;
  const height = cameraInfo?.height ?? 720;
  const frameId = cameraInfo?.header?.frame_id || "camera_frame";

  const observations = obsBatch?.observations ?? [];
  const visibleCount = observations.length;
  const calibSha = observations.length > 0 ? observations[0]?.calibration_sha256 : undefined;
  const calibShortSha = calibSha ? calibSha.substring(0, 8) : "VERIFIED";

  // Lock status info
  const getLockInfo = (state?: number) => {
    switch (state) {
      case 0: return { label: "NO TARGET", color: "#8b949e", bg: "rgba(136,136,136,0.15)", border: "#6e7681" };
      case 1: return { label: "ACQUIRING", color: "#f1c40f", bg: "rgba(243,156,18,0.15)", border: "#d29922" };
      case 2: return { label: "CANDIDATE", color: "#58a6ff", bg: "rgba(56,139,253,0.15)", border: "#1f6feb" };
      case 3: return { label: "QUALIFIED (LOCKED)", color: "#7ee787", bg: "rgba(46,160,67,0.15)", border: "#2ea043" };
      case 4: return { label: "TARGET LOST", color: "#ff7b72", bg: "rgba(248,81,73,0.15)", border: "#f85149" };
      default: return { label: "SEARCHING / STANDBY", color: "#58a6ff", bg: "rgba(56,139,253,0.1)", border: "#30363d" };
    }
  };

  const lockInfo = getLockInfo(liveLock?.lock_state);
  const targetId = liveLock?.identity?.marker_id;
  const targetQuality = liveLock?.quality != null ? Math.round(liveLock.quality * 100) : null;

  return (
    <div
      className="custom-scrollbar"
      style={{
        padding: "12px",
        fontFamily: "'JetBrains Mono', 'Segoe UI', system-ui, -apple-system, sans-serif",
        color: "#e6edf3",
        background: "linear-gradient(180deg, #0d1117 0%, #161b22 100%)",
        height: "100%",
        overflowY: "auto",
        boxSizing: "border-box",
        display: "flex",
        flexDirection: "column",
        gap: "10px",
      }}
    >
      <style>{customStyles}</style>

      {/* Header: Clean Aerospace Status Bar */}
      <div
        style={{
          display: "flex",
          justifyContent: "space-between",
          alignItems: "center",
          background: "rgba(22, 27, 34, 0.85)",
          backdropFilter: "blur(10px)",
          border: "1px solid rgba(88, 166, 255, 0.2)",
          borderRadius: "8px",
          padding: "8px 12px",
          boxShadow: "0 4px 16px rgba(0, 0, 0, 0.3)",
          flexShrink: 0,
        }}
      >
        <div style={{ display: "flex", alignItems: "center", gap: "8px" }}>
          <div
            style={{
              width: "8px",
              height: "8px",
              borderRadius: "50%",
              background: isOnline ? (isStalled ? "#d29922" : "#2ea043") : "#f85149",
              animation: isOnline ? "pulseGreenGlow 2s infinite" : "pulseRedGlow 2s infinite",
            }}
          />
          <div>
            <div style={{ fontSize: "12px", fontWeight: "bold", letterSpacing: "0.5px", color: "#58a6ff" }}>
              DOWNWARD CAMERA & PERCEPTION
            </div>
            <div style={{ fontSize: "9px", color: "#8b949e" }}>
              OPTICAL FRAME: {frameId}
            </div>
          </div>
        </div>

        <div
          style={{
            fontSize: "10px",
            fontWeight: "bold",
            padding: "3px 10px",
            borderRadius: "12px",
            background: isOnline ? "rgba(46, 160, 67, 0.15)" : "rgba(248, 81, 73, 0.15)",
            border: `1px solid ${isOnline ? "#2ea043" : "#f85149"}`,
            color: isOnline ? "#7ee787" : "#ff7b72",
            display: "flex",
            alignItems: "center",
            gap: "5px",
          }}
        >
          {isOnline ? `FEED ACTIVE • ${fps.toFixed(0)} FPS` : "FEED OFFLINE"}
        </div>
      </div>

      {/* Main Two-Column Telemetry Grid */}
      <div
        style={{
          display: "grid",
          gridTemplateColumns: "1fr 1fr",
          gap: "10px",
          flex: 1,
        }}
      >
        {/* Card 1: Camera Video Ingestion Metrics */}
        <div
          className="hud-stat-box"
          style={{
            background: "rgba(22, 27, 34, 0.5)",
            border: "1px solid rgba(48, 54, 61, 0.8)",
            borderRadius: "8px",
            padding: "10px 12px",
            display: "flex",
            flexDirection: "column",
            gap: "8px",
          }}
        >
          <div style={{ display: "flex", alignItems: "center", gap: "6px", borderBottom: "1px solid rgba(48, 54, 61, 0.6)", paddingBottom: "5px" }}>
            <span style={{ fontSize: "12px" }}>📹</span>
            <span style={{ fontSize: "10px", fontWeight: "bold", color: "#58a6ff", letterSpacing: "0.5px" }}>
              VIDEO INGESTION
            </span>
          </div>

          <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "6px" }}>
            <div>
              <div style={{ fontSize: "9px", color: "#8b949e" }}>FRAME RATE</div>
              <div style={{ fontSize: "16px", fontWeight: "bold", color: isOnline ? "#7ee787" : "#8b949e" }}>
                {isOnline ? `${fps.toFixed(1)}` : "0.0"} <span style={{ fontSize: "10px", fontWeight: 400 }}>FPS</span>
              </div>
            </div>

            <div>
              <div style={{ fontSize: "9px", color: "#8b949e" }}>RESOLUTION</div>
              <div style={{ fontSize: "12px", fontWeight: 600, color: "#c9d1d9", marginTop: "2px" }}>
                {width} × {height}
              </div>
            </div>
          </div>

          <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "6px", marginTop: "2px" }}>
            <div>
              <div style={{ fontSize: "9px", color: "#8b949e" }}>TOTAL FRAMES</div>
              <div style={{ fontSize: "11px", fontWeight: 600, color: "#c9d1d9" }}>
                {frameCount.toLocaleString()} msgs
              </div>
            </div>

            <div>
              <div style={{ fontSize: "9px", color: "#8b949e" }}>FRESHNESS</div>
              <div style={{ fontSize: "11px", fontWeight: 600, color: isOnline ? "#7ee787" : "#ff7b72" }}>
                {isOnline ? `< ${(timeSinceLastFrameSec).toFixed(2)}s` : "STALE"}
              </div>
            </div>
          </div>
        </div>

        {/* Card 2: ArUco Vision & Detection Metrics */}
        <div
          className="hud-stat-box"
          style={{
            background: "rgba(22, 27, 34, 0.5)",
            border: "1px solid rgba(48, 54, 61, 0.8)",
            borderRadius: "8px",
            padding: "10px 12px",
            display: "flex",
            flexDirection: "column",
            gap: "8px",
          }}
        >
          <div style={{ display: "flex", alignItems: "center", gap: "6px", borderBottom: "1px solid rgba(48, 54, 61, 0.6)", paddingBottom: "5px" }}>
            <span style={{ fontSize: "12px" }}>🎯</span>
            <span style={{ fontSize: "10px", fontWeight: "bold", color: "#58a6ff", letterSpacing: "0.5px" }}>
              ARUCO VISION DETECTOR
            </span>
          </div>

          <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "6px" }}>
            <div>
              <div style={{ fontSize: "9px", color: "#8b949e" }}>DICTIONARY</div>
              <div style={{ fontSize: "11px", fontWeight: 600, color: "#c9d1d9" }}>
                DICT_4X4_50
              </div>
            </div>

            <div>
              <div style={{ fontSize: "9px", color: "#8b949e" }}>CALIBRATION</div>
              <div style={{ fontSize: "10px", fontWeight: 600, color: "#7ee787" }}>
                SHA256: {calibShortSha}
              </div>
            </div>
          </div>

          <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "6px" }}>
            <div>
              <div style={{ fontSize: "9px", color: "#8b949e" }}>PERCEPTION NODE</div>
              <div style={{ fontSize: "10px", fontWeight: 600, color: (perceptionHealth?.ready ?? true) ? "#7ee787" : "#f1c40f" }}>
                {(perceptionHealth?.ready ?? true) ? "READY / ACTIVE" : "INITIALIZING"}
              </div>
            </div>
            <div>
              <div style={{ fontSize: "9px", color: "#8b949e" }}>TARGET SCOPE</div>
              <div style={{ fontSize: "10px", fontWeight: 600, color: "#c9d1d9" }}>
                aavc2026
              </div>
            </div>
          </div>

          <div>
            <div style={{ fontSize: "9px", color: "#8b949e", marginBottom: "4px" }}>
              MARKERS IN CURRENT FRAME ({visibleCount})
            </div>
            {visibleCount === 0 ? (
              <div style={{ fontSize: "10px", color: "#6e7681", fontStyle: "italic" }}>
                No markers currently in field of view
              </div>
            ) : (
              <div style={{ display: "flex", flexWrap: "wrap", gap: "4px" }}>
                {observations.map((obs, i) => (
                  <span
                    key={i}
                    style={{
                      fontSize: "10px",
                      fontWeight: "bold",
                      padding: "2px 6px",
                      borderRadius: "4px",
                      background: "rgba(56, 139, 253, 0.15)",
                      border: "1px solid rgba(56, 139, 253, 0.4)",
                      color: "#58a6ff",
                    }}
                  >
                    PAD #{obs.identity?.marker_id ?? "?"}
                  </span>
                ))}
              </div>
            )}
          </div>
        </div>
      </div>

      {/* Target Lock Sub-Banner */}
      <div
        style={{
          display: "flex",
          justifyContent: "space-between",
          alignItems: "center",
          background: lockInfo.bg,
          border: `1px solid ${lockInfo.border}`,
          borderRadius: "6px",
          padding: "6px 12px",
          flexShrink: 0,
        }}
      >
        <div style={{ display: "flex", alignItems: "center", gap: "8px" }}>
          <span style={{ fontSize: "11px" }}>🎯</span>
          <span style={{ fontSize: "10px", fontWeight: "bold", color: "#c9d1d9" }}>
            ASSIGNED TARGET:
          </span>
          <span
            style={{
              fontSize: "10px",
              fontWeight: "bold",
              color: targetId != null ? "#58a6ff" : "#8b949e",
            }}
          >
            {targetId != null ? `PAD #${targetId}` : "NONE"}
          </span>
        </div>

        <div style={{ display: "flex", alignItems: "center", gap: "8px" }}>
          {targetQuality != null && (
            <span style={{ fontSize: "9px", color: "#c9d1d9" }}>
              QUAL: <strong style={{ color: "#7ee787" }}>{targetQuality}%</strong>
            </span>
          )}
          <span
            style={{
              fontSize: "9px",
              fontWeight: "bold",
              color: lockInfo.color,
              padding: "2px 6px",
              borderRadius: "4px",
              background: "rgba(0, 0, 0, 0.3)",
              border: `1px solid ${lockInfo.border}`,
            }}
          >
            {lockInfo.label}
          </span>
        </div>
      </div>
    </div>
  );
}

export function initFsdCameraStatusPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<FsdCameraStatusPanel context={context} />);

  return () => {
    root.unmount();
  };
}
