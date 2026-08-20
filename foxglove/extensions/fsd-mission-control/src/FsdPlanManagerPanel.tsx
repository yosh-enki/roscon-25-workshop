import { MessageEvent, PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useState, useRef } from "react";
import { createRoot } from "react-dom/client";

interface WorkingPlanStatusMsg {
  state?: number;
  working_plan_id?: string;
  map_id?: string;
  scenario_id?: string;
  source_artifact_sha256?: string;
  generation?: number;
  durability_state?: number;
}

interface LogEntry {
  id: string;
  time: string;
  type: "info" | "success" | "error" | "warn";
  text: string;
}

const customStyles = `
  .plan-btn {
    transition: all 0.15s cubic-bezier(0.4, 0, 0.2, 1);
    font-family: inherit;
  }
  .plan-btn:hover:not(:disabled) {
    transform: translateY(-1px);
    filter: brightness(1.15);
  }
  .plan-btn:active:not(:disabled) {
    transform: translateY(1px);
    filter: brightness(0.9);
  }
  .plan-chip {
    transition: all 0.15s ease;
  }
  .plan-chip:hover {
    transform: scale(1.02);
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
`;

function FsdPlanManagerPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [selectedPlanName, setSelectedPlanName] = useState<string>("aavc2026_mission.plan");
  const [workingPlan, setWorkingPlan] = useState<WorkingPlanStatusMsg | undefined>();
  const [isCalling, setIsCalling] = useState<boolean>(false);
  const [logs, setLogs] = useState<LogEntry[]>([]);
  const fileInputRef = useRef<HTMLInputElement | null>(null);

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
          if (m.topic === "/full_self_driving/plan/working_status") {
            setWorkingPlan(m.message as WorkingPlanStatusMsg);
          }
        }
      }
    };

    context.watch("currentFrame");
    context.watch("topics");

    context.subscribe([
      { topic: "/full_self_driving/plan/working_status" },
    ]);
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const handleSelectPlan = async (planName: string) => {
    setSelectedPlanName(planName);
    setIsCalling(true);
    addLog("info", `Selecting plan: ${planName}...`);

    try {
      const callFn = (context as unknown as { callService?: (name: string, req: unknown) => Promise<unknown> }).callService;
      if (typeof callFn !== "function") {
        throw new Error("Foxglove service caller not available");
      }

      const res = (await callFn("/full_self_driving/select_plan_artifact", {
        request_id: `select_plan_${Date.now()}`,
        artifact_id: planName,
        expected_selection_revision: 0,
      })) as { accepted: boolean };

      if (res.accepted) {
        addLog("success", `Plan '${planName}' selected and committed successfully!`);
      } else {
        throw new Error("Plan selection rejected by flight runtime");
      }
    } catch (err: unknown) {
      const msg = err instanceof Error ? err.message : String(err);
      addLog("error", `Plan selection failed: ${msg}`);
    } finally {
      setIsCalling(false);
    }
  };

  const handleFileUpload = async (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;

    setIsCalling(true);
    addLog("info", `Reading file '${file.name}' (${file.size} bytes)...`);

    try {
      const buffer = await file.arrayBuffer();
      const bytes = Array.from(new Uint8Array(buffer));
      const safeName = file.name;

      const callFn = (context as unknown as { callService?: (name: string, req: unknown) => Promise<unknown> }).callService;
      if (typeof callFn !== "function") {
        throw new Error("Foxglove service caller not available");
      }

      addLog("info", `Uploading plan artifact '${safeName}'...`);
      const uploadRes = (await callFn("/full_self_driving/upload_plan_artifact", {
        request_id: `upload_${Date.now()}`,
        safe_name: safeName,
        content: bytes,
        expected_selection_revision: 0,
      })) as { accepted: boolean; artifact?: { artifact_id: string } };

      if (uploadRes.accepted && uploadRes.artifact?.artifact_id) {
        const artId = uploadRes.artifact.artifact_id;
        setSelectedPlanName(safeName);
        addLog("success", `Plan uploaded: ${artId}`);

        // Automatically select the newly uploaded plan
        await callFn("/full_self_driving/select_plan_artifact", {
          request_id: `select_${Date.now()}`,
          artifact_id: artId,
          expected_selection_revision: 0,
        });
        addLog("success", `Plan '${safeName}' selected and committed!`);
      } else {
        throw new Error("Plan upload was not accepted by flight runtime");
      }
    } catch (err: unknown) {
      const msg = err instanceof Error ? err.message : String(err);
      addLog("error", `Plan upload failed: ${msg}`);
    } finally {
      setIsCalling(false);
      if (fileInputRef.current) {
        fileInputRef.current.value = "";
      }
    }
  };

  const getPlanStateText = (state?: number) => {
    switch (state) {
      case 1: return { text: "READY", color: "#3fb950", bg: "rgba(46,160,67,0.15)" };
      case 2: return { text: "SEARCHING", color: "#58a6ff", bg: "rgba(88,166,255,0.15)" };
      case 3: return { text: "COMPLETED", color: "#bc8cff", bg: "rgba(188,140,255,0.15)" };
      case 4: return { text: "INVALID", color: "#f85149", bg: "rgba(248,81,73,0.15)" };
      case 5: return { text: "RECOVERY", color: "#d29922", bg: "rgba(210,153,34,0.15)" };
      default: return { text: "LOADED", color: "#79c0ff", bg: "rgba(88,166,255,0.15)" };
    }
  };

  const planStateInfo = getPlanStateText(workingPlan?.state);

  return (
    <div
      className="custom-scrollbar"
      style={{
        padding: "14px",
        fontFamily: "'JetBrains Mono', 'Segoe UI', system-ui, -apple-system, sans-serif",
        color: "#e6edf3",
        background: "linear-gradient(180deg, #0d1117 0%, #161b22 100%)",
        height: "100%",
        overflowY: "auto",
        boxSizing: "border-box",
      }}
    >
      <style>{customStyles}</style>

      {/* Header */}
      <div
        style={{
          display: "flex",
          justifyContent: "space-between",
          alignItems: "center",
          background: "rgba(22, 27, 34, 0.8)",
          border: "1px solid rgba(88, 166, 255, 0.2)",
          borderRadius: "8px",
          padding: "8px 12px",
          marginBottom: "12px",
        }}
      >
        <div style={{ fontSize: "13px", fontWeight: "bold", color: "#58a6ff", letterSpacing: "0.5px" }}>
          FLIGHT PLAN MANAGER
        </div>
        <span
          style={{
            fontSize: "10px",
            padding: "2px 8px",
            borderRadius: "4px",
            background: planStateInfo.bg,
            color: planStateInfo.color,
            border: `1px solid ${planStateInfo.color}`,
            fontWeight: "bold",
          }}
        >
          {planStateInfo.text}
        </span>
      </div>

      {/* Active Working Plan Card */}
      <div
        style={{
          background: "rgba(22, 27, 34, 0.8)",
          border: "1px solid #30363d",
          borderRadius: "8px",
          padding: "10px",
          marginBottom: "12px",
          fontSize: "11px",
        }}
      >
        <div style={{ color: "#8b949e", marginBottom: "4px" }}>Active Plan ID:</div>
        <div style={{ color: "#79c0ff", fontWeight: "bold", marginBottom: "6px", wordBreak: "break-all" }}>
          {workingPlan?.working_plan_id || selectedPlanName || "aavc2026_mission.plan"}
        </div>
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "6px", color: "#8b949e", fontSize: "10px" }}>
          <div>Map: <b style={{ color: "#c9d1d9" }}>{workingPlan?.map_id || "kmitl_airfield"}</b></div>
          <div>Scenario: <b style={{ color: "#c9d1d9" }}>{workingPlan?.scenario_id || "default_scenario"}</b></div>
        </div>
      </div>

      {/* Plan Selection Buttons */}
      <div
        style={{
          background: "rgba(22, 27, 34, 0.8)",
          border: "1px solid #30363d",
          borderRadius: "8px",
          padding: "10px",
          marginBottom: "12px",
        }}
      >
        <div style={{ fontSize: "11px", color: "#8b949e", marginBottom: "6px" }}>Select / Switch Plan:</div>
        <div style={{ display: "flex", gap: "6px", marginBottom: "8px" }}>
          <button
            type="button"
            className="plan-chip"
            onClick={() => handleSelectPlan("aavc2026_mission.plan")}
            disabled={isCalling}
            style={{
              flex: 1,
              padding: "8px",
              borderRadius: "6px",
              fontSize: "11px",
              fontWeight: selectedPlanName === "aavc2026_mission.plan" ? "bold" : "normal",
              background: selectedPlanName === "aavc2026_mission.plan" ? "rgba(88, 166, 255, 0.2)" : "rgba(33, 38, 45, 0.8)",
              border: selectedPlanName === "aavc2026_mission.plan" ? "1px solid #58a6ff" : "1px solid #30363d",
              color: selectedPlanName === "aavc2026_mission.plan" ? "#79c0ff" : "#c9d1d9",
              cursor: isCalling ? "not-allowed" : "pointer",
            }}
          >
            aavc2026_mission.plan
          </button>
        </div>

        <input
          type="file"
          accept=".plan,.json"
          ref={fileInputRef}
          onChange={handleFileUpload}
          style={{ display: "none" }}
        />

        <button
          type="button"
          className="plan-btn"
          onClick={() => fileInputRef.current?.click()}
          disabled={isCalling}
          style={{
            width: "100%",
            padding: "8px",
            borderRadius: "6px",
            fontSize: "11px",
            background: "rgba(56, 139, 253, 0.15)",
            border: "1px dashed #58a6ff",
            color: "#58a6ff",
            cursor: isCalling ? "not-allowed" : "pointer",
            fontWeight: "bold",
          }}
        >
          UPLOAD NEW .PLAN FILE
        </button>
      </div>

      {/* Plan Logs */}
      <div
        style={{
          background: "#0d1117",
          border: "1px solid #30363d",
          borderRadius: "6px",
          padding: "8px",
        }}
      >
        <div style={{ fontSize: "10px", fontWeight: "bold", color: "#8b949e", marginBottom: "4px" }}>
          PLAN AUDIT LOGS
        </div>
        <div className="custom-scrollbar" style={{ maxHeight: "80px", overflowY: "auto", fontSize: "10px", lineHeight: "1.4" }}>
          {logs.length === 0 ? (
            <div style={{ color: "#484f58", fontStyle: "italic" }}>No plan actions yet.</div>
          ) : (
            logs.map((log) => (
              <div
                key={log.id}
                style={{
                  marginBottom: "3px",
                  color: log.type === "success" ? "#7ee787" : log.type === "error" ? "#ff7b72" : "#8b949e",
                }}
              >
                <span style={{ color: "#484f58", marginRight: "4px" }}>[{log.time}]</span>
                {log.text}
              </div>
            ))
          )}
        </div>
      </div>
    </div>
  );
}

export function initFsdPlanManagerPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<FsdPlanManagerPanel context={context} />);

  return () => {
    root.unmount();
  };
}
