import type { DigitId, SimulationState } from './types';

export type ExportedSimulationState = Omit<SimulationState, 'solverDiagnostics' | 'analyticSolveSteps'>;

export interface SimulationExport {
  schema: 'linkage-simulator-state';
  schemaVersion: 1;
  exportedAtUtc: string;
  units: {
    length: 'mm';
    angle: 'rad';
    mass: 'kg';
    moment: 'N·m';
    acceleration: 'm/s²';
  };
  state: ExportedSimulationState;
}

export interface MultiDigitSimulationExport {
  schema: 'linkage-simulator-multidigit-state';
  schemaVersion: 2;
  exportedAtUtc: string;
  units: SimulationExport['units'];
  overallHandScale: number;
  activeDigitId: DigitId;
  digits: Record<DigitId, ExportedSimulationState>;
}

function persistentState(state: SimulationState): ExportedSimulationState {
  const clone = structuredClone(state);
  const { solverDiagnostics: _solverDiagnostics, analyticSolveSteps: _analyticSolveSteps, ...persistent } = clone;
  return persistent;
}

export function createSimulationExport(
  state: SimulationState,
  exportedAt: Date = new Date(),
): SimulationExport {
  return {
    schema: 'linkage-simulator-state',
    schemaVersion: 1,
    exportedAtUtc: exportedAt.toISOString(),
    units: {
      length: 'mm',
      angle: 'rad',
      mass: 'kg',
      moment: 'N·m',
      acceleration: 'm/s²',
    },
    state: persistentState(state),
  };
}

export function createMultiDigitSimulationExport(
  digitStates: Record<DigitId, SimulationState>,
  overallHandScale: number,
  activeDigitId: DigitId,
  exportedAt: Date = new Date(),
): MultiDigitSimulationExport {
  return {
    schema: 'linkage-simulator-multidigit-state',
    schemaVersion: 2,
    exportedAtUtc: exportedAt.toISOString(),
    units: {
      length: 'mm',
      angle: 'rad',
      mass: 'kg',
      moment: 'N·m',
      acceleration: 'm/s²',
    },
    overallHandScale,
    activeDigitId,
    digits: Object.fromEntries(
      Object.entries(digitStates).map(([digitId, state]) => [digitId, persistentState(state)]),
    ) as Record<DigitId, ExportedSimulationState>,
  };
}
