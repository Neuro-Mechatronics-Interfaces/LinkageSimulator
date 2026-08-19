import { createMultiDigitSimulationExport, type AppStore } from '../model';

export function downloadSimulationJson(store: AppStore, exportedAt: Date = new Date()): void {
  const payload = createMultiDigitSimulationExport(
    store.digitStates,
    store.overallHandScale,
    store.activeDigitId,
    exportedAt,
  );
  const blob = new Blob([`${JSON.stringify(payload, null, 2)}\n`], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement('a');
  const timestamp = exportedAt.toISOString().replace(/[:.]/g, '-');
  anchor.href = url;
  anchor.download = `linkage-simulator-all-digits-${timestamp}.json`;
  anchor.click();
  URL.revokeObjectURL(url);
}
