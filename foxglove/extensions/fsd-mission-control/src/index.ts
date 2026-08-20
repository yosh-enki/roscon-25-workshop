import { ExtensionContext } from "@foxglove/extension";
import { initFsdMissionControlPanel } from "./FsdMissionControlPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "fsd-mission-control",
    initPanel: initFsdMissionControlPanel,
  });
}
