import type { DigitId, SimulationState } from './types';

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
  state: SimulationState;
}

export interface MultiDigitSimulationExport {
  schema: 'linkage-simulator-multidigit-state';
  schemaVersion: 2;
  exportedAtUtc: string;
  units: SimulationExport['units'];
  overallHandScale: number;
  activeDigitId: DigitId;
  digits: Record<DigitId, SimulationState>;
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
    state: structuredClone(state),
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
    digits: structuredClone(digitStates),
  };
}
