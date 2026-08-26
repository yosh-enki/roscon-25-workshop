import { ExtensionContext } from "@foxglove/extension";
import { initFsdMissionControlPanel } from "./FsdMissionControlPanel";
import { initFsdPlanManagerPanel } from "./FsdPlanManagerPanel";
import { initFsdPadRegistryPanel } from "./FsdPadRegistryPanel";
import { initFsdCameraStatusPanel } from "./FsdCameraStatusPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "fsd-mission-control",
    initPanel: initFsdMissionControlPanel,
  });

  extensionContext.registerPanel({
    name: "fsd-plan-manager",
    initPanel: initFsdPlanManagerPanel,
  });

  extensionContext.registerPanel({
    name: "fsd-pad-registry",
    initPanel: initFsdPadRegistryPanel,
  });

  extensionContext.registerPanel({
    name: "fsd-camera-status",
    initPanel: initFsdCameraStatusPanel,
  });
}


