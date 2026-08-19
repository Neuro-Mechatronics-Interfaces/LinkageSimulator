import { createSimulationExport, type SimulationState } from '../model';

export function downloadSimulationJson(state: SimulationState, exportedAt: Date = new Date()): void {
  const payload = createSimulationExport(state, exportedAt);
  const blob = new Blob([`${JSON.stringify(payload, null, 2)}\n`], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement('a');
  const timestamp = exportedAt.toISOString().replace(/[:.]/g, '-');
  anchor.href = url;
  anchor.download = `linkage-simulator-${timestamp}.json`;
  anchor.click();
  URL.revokeObjectURL(url);
}
