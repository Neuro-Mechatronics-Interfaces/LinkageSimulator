import type { SimulationState } from './types';

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
