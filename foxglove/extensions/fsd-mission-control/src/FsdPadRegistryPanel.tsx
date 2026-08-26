import { MessageEvent, PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

interface PadRecordMsg {
  identity?: {
    marker_id?: number;
    dictionary?: string;
    target_namespace?: string;
  };
  map_id?: string;
  scenario_id?: string;
  latitude_deg?: number;
  longitude_deg?: number;
  altitude_m?: number;
  uncertainty_m?: number;
  quality?: number;
  observation_count?: number;
  last_observed_monotonic_ns?: number | bigint;
  calibration_sha256?: string;
}

interface PadRegistrySnapshotMsg {
  map_id?: string;
  scenario_id?: string;
  revision?: number | bigint;
  records?: PadRecordMsg[];
  origin_state?: number;
  durability_state?: number;
  backup_state?: number;
}

const customStyles = `
  @keyframes pulseGreen {
    0% { box-shadow: 0 0 4px rgba(46, 204, 113, 0.4); }
    50% { box-shadow: 0 0 12px rgba(46, 204, 113, 0.8); }
    100% { box-shadow: 0 0 4px rgba(46, 204, 113, 0.4); }
  }
  .pad-row {
    transition: background 0.15s ease, border-color 0.15s ease;
  }
  .pad-row:hover {
    background: rgba(88, 166, 255, 0.08) !important;
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

function FsdPadRegistryPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [snapshot, setSnapshot] = useState<PadRegistrySnapshotMsg | undefined>();
  const [lastUpdateStr, setLastUpdateStr] = useState<string>("--:--:--");
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);

      if (renderState.currentFrame) {
        for (const msg of renderState.currentFrame) {
          const m = msg as MessageEvent<unknown>;
          if (m.topic === "/full_self_driving/pad_registry") {
            setSnapshot(m.message as PadRegistrySnapshotMsg);
            setLastUpdateStr(new Date().toLocaleTimeString());
          }
        }
      }
    };

    context.watch("currentFrame");
    context.watch("topics");

    context.subscribe([
      { topic: "/full_self_driving/pad_registry" },
    ]);
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const rawRecords = snapshot?.records ?? [];
  // Sort records deterministically by marker ID ascending
  const records = [...rawRecords].sort((a, b) => {
    const idA = a.identity?.marker_id ?? 0;
    const idB = b.identity?.marker_id ?? 0;
    return idA - idB;
  });

  const recordCount = records.length;
  const mapId = snapshot?.map_id || "kmitl_airfield";
  const scenarioId = snapshot?.scenario_id || "default_scenario";
  const revision = snapshot?.revision != null ? String(snapshot.revision) : "1";
  const hasData = snapshot != null && recordCount > 0;

  const getQualityColor = (q: number) => {
    if (q >= 0.8) return { text: "#7ee787", bar: "#2ea043", bg: "rgba(46, 160, 67, 0.2)" };
    if (q >= 0.5) return { text: "#e3b341", bar: "#d29922", bg: "rgba(210, 153, 34, 0.2)" };
    return { text: "#ff7b72", bar: "#f85149", bg: "rgba(248, 81, 73, 0.2)" };
  };

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

      {/* Header: Clean Aerospace HUD Bar */}
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
              background: hasData ? "#2ea043" : "#8b949e",
              animation: hasData ? "pulseGreen 2s infinite" : "none",
            }}
          />
          <div>
            <div style={{ fontSize: "12px", fontWeight: "bold", letterSpacing: "0.5px", color: "#58a6ff" }}>
              PAD REGISTRY
            </div>
            <div style={{ fontSize: "9px", color: "#8b949e" }}>
              SCOPE: {mapId} / {scenarioId}
            </div>
          </div>
        </div>

        <div style={{ display: "flex", alignItems: "center", gap: "6px" }}>
          <div
            style={{
              fontSize: "10px",
              fontWeight: 600,
              padding: "3px 8px",
              borderRadius: "12px",
              background: recordCount > 0 ? "rgba(56, 139, 253, 0.15)" : "rgba(110, 118, 129, 0.15)",
              border: `1px solid ${recordCount > 0 ? "rgba(56, 139, 253, 0.4)" : "rgba(110, 118, 129, 0.3)"}`,
              color: recordCount > 0 ? "#58a6ff" : "#8b949e",
            }}
          >
            {recordCount} {recordCount === 1 ? "PAD" : "PADS"}
          </div>
          <div
            style={{
              fontSize: "9px",
              color: "#8b949e",
              padding: "3px 6px",
              borderRadius: "4px",
              background: "rgba(255, 255, 255, 0.04)",
              border: "1px solid rgba(255, 255, 255, 0.08)",
            }}
          >
            REV: {revision}
          </div>
        </div>
      </div>

      {/* Discovered Pads Table */}
      {!hasData ? (
        <div
          style={{
            flex: 1,
            display: "flex",
            flexDirection: "column",
            alignItems: "center",
            justifyContent: "center",
            background: "rgba(22, 27, 34, 0.4)",
            border: "1px dashed rgba(88, 166, 255, 0.2)",
            borderRadius: "8px",
            padding: "24px 16px",
            textAlign: "center",
          }}
        >
          <div style={{ fontSize: "28px", marginBottom: "8px", filter: "grayscale(30%)" }}>🛰️</div>
          <div style={{ fontSize: "12px", fontWeight: "bold", color: "#c9d1d9", marginBottom: "4px" }}>
            No Landing Pads Discovered Yet
          </div>
          <div style={{ fontSize: "10px", color: "#8b949e", maxWidth: "260px", lineHeight: "1.4" }}>
            Awaiting ArUco visual observations from downward camera feed during survey flight...
          </div>
        </div>
      ) : (
        <div
          style={{
            flex: 1,
            background: "rgba(22, 27, 34, 0.5)",
            border: "1px solid rgba(48, 54, 61, 0.8)",
            borderRadius: "8px",
            overflow: "hidden",
            display: "flex",
            flexDirection: "column",
          }}
        >
          {/* Table Header */}
          <div
            style={{
              display: "grid",
              gridTemplateColumns: "70px 1fr 65px 75px 65px",
              padding: "7px 10px",
              background: "rgba(13, 17, 23, 0.9)",
              borderBottom: "1px solid rgba(48, 54, 61, 0.8)",
              fontSize: "9px",
              fontWeight: "bold",
              color: "#8b949e",
              letterSpacing: "0.5px",
            }}
          >
            <div>PAD</div>
            <div>COORDINATES (WGS-84)</div>
            <div style={{ textAlign: "right" }}>ALT (AMSL)</div>
            <div style={{ textAlign: "center" }}>QUALITY</div>
            <div style={{ textAlign: "right" }}>ACCURACY</div>
          </div>

          {/* Table Rows */}
          <div className="custom-scrollbar" style={{ overflowY: "auto", flex: 1 }}>
            {records.map((pad, idx) => {
              const markerId = pad.identity?.marker_id ?? idx;
              const lat = pad.latitude_deg != null ? pad.latitude_deg.toFixed(7) : "0.0000000";
              const lon = pad.longitude_deg != null ? pad.longitude_deg.toFixed(7) : "0.0000000";
              const alt = pad.altitude_m != null ? `${pad.altitude_m.toFixed(2)} m` : "--";
              const rawQuality = pad.quality ?? 0;
              const qualityPct = Math.round(rawQuality * 100);
              const qColors = getQualityColor(rawQuality);
              const uncertainty = pad.uncertainty_m != null ? `±${pad.uncertainty_m.toFixed(2)}m` : "--";
              const obsCount = pad.observation_count != null ? Number(pad.observation_count) : 0;

              return (
                <div
                  key={`${markerId}_${idx}`}
                  className="pad-row"
                  style={{
                    display: "grid",
                    gridTemplateColumns: "70px 1fr 65px 75px 65px",
                    alignItems: "center",
                    padding: "8px 10px",
                    borderBottom: idx === records.length - 1 ? "none" : "1px solid rgba(48, 54, 61, 0.4)",
                    background: idx % 2 === 0 ? "rgba(22, 27, 34, 0.2)" : "rgba(13, 17, 23, 0.2)",
                    fontSize: "11px",
                  }}
                >
                  {/* Pad ID Badge */}
                  <div style={{ display: "flex", alignItems: "center", gap: "6px" }}>
                    <span
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
                      PAD #{markerId}
                    </span>
                  </div>

                  {/* GPS Coordinates & Obs Count */}
                  <div>
                    <div style={{ fontSize: "11px", fontWeight: 600, color: "#e6edf3", letterSpacing: "0.2px" }}>
                      {lat}°, {lon}°
                    </div>
                    <div style={{ fontSize: "9px", color: "#6e7681", marginTop: "1px" }}>
                      {obsCount} observation{obsCount === 1 ? "" : "s"}
                    </div>
                  </div>

                  {/* Altitude */}
                  <div style={{ textAlign: "right", color: "#c9d1d9", fontWeight: 500, fontSize: "10px" }}>
                    {alt}
                  </div>

                  {/* Quality Mini Bar */}
                  <div style={{ padding: "0 6px" }}>
                    <div style={{ display: "flex", justifyContent: "space-between", fontSize: "9px", color: qColors.text, marginBottom: "2px", fontWeight: "bold" }}>
                      <span>QUAL</span>
                      <span>{qualityPct}%</span>
                    </div>
                    <div
                      style={{
                        height: "4px",
                        borderRadius: "2px",
                        background: "rgba(255, 255, 255, 0.08)",
                        overflow: "hidden",
                      }}
                    >
                      <div
                        style={{
                          height: "100%",
                          width: `${Math.min(100, Math.max(0, qualityPct))}%`,
                          background: qColors.bar,
                          borderRadius: "2px",
                        }}
                      />
                    </div>
                  </div>

                  {/* Accuracy Tolerance */}
                  <div style={{ textAlign: "right", color: "#8b949e", fontSize: "10px" }}>
                    {uncertainty}
                  </div>
                </div>
              );
            })}
          </div>
        </div>
      )}

      {/* Footer Status Sub-bar */}
      <div
        style={{
          display: "flex",
          justifyContent: "space-between",
          alignItems: "center",
          fontSize: "9px",
          color: "#6e7681",
          padding: "0 4px",
          flexShrink: 0,
        }}
      >
        <div>DURABILITY: SYNCED</div>
        <div>LAST UPDATED: {lastUpdateStr}</div>
      </div>
    </div>
  );
}

export function initFsdPadRegistryPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<FsdPadRegistryPanel context={context} />);

  return () => {
    root.unmount();
  };
}
